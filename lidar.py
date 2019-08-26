from rplidar import RPLidar
from concurrent.futures import ThreadPoolExecutor
from threading import Thread, Lock, Event
from collections import deque
import time
import sys
import paho.mqtt.client as mqtt
import os
import copy
import logging
import math
import Constant as c


class Lidar:
    def __init__(self):

        """
            __________________________________
            |     |      |      |      |      |
            |     |      |      |      |      |
            |_____|______|______|______|______|
        """

        self._lidarSecReading = {}
        self._lidarNextSecReading = {}
        self._currTimeReading = None
        self._lidarReading = list([(c.INIT_TIME, {}) for i in range(c.RECORD_CONSTANT)])
        self._jetsonMsgReceived = False
        self._jetsonMsgCv = Event()
        self._jetsonMsgCv.set()
        self._client = None
        self._subChannel = "jetson"
        self._pubChannel = "intercept"


    def roundAngle(self, dataReading):
        """ Rounds the angle measurement to the nearest 0.5

        Parameters:
        ------------
        dataReading : list
            A single raw data reading from the lidar scan.
            The reading includes.
            [scan quality, angle (degrees), the depth (mm)]

        -------------
        Returns: list
            A list of [ rounded angle, the depth ]. The scan
            quality is removed.
        """

        roundedAngle = round(dataReading[1] * c.ROUND_DECIMAL_ANGLE)/c.ROUND_DECIMAL_ANGLE
        singleEntry = (roundedAngle, dataReading[2])
        return singleEntry




    def parseLidarData(self, scan, measuredTime):
        """ Processes the raw lidar scan data, and stores
        the data in the |lidarSecReading| dict for the current second.
        When the time with a new second unit is given, the previous |lidarSecReading|
        is put into a fixed |_lidarReading| cache.

        Parameters:
        ------------
        scan : list
            A single raw data reading from the lidar scan.
            ===> [scan quality, angle (degrees), the depth (mm)]

        measuredTime : float
            unixTimeStamp (given by the this program and NOT from the lidar)
        """
        if self._currTimeReading != None and abs(measuredTime - self._currTimeReading):
            self._lidarSecReading = {}
        self._currTimeReading = measuredTime
        mapIndex = measuredTime % c.RECORD_CONSTANT
        lidarData = map(self.roundAngle, scan)
        self._lidarSecReading.update(lidarData)
        self._lidarReading[mapIndex] = (measuredTime, self._lidarSecReading)



    def stopLidarThread(self, stop):
        """ Runs a separate thread that takes in user input to
        exit the program safely

        Parameters:
        ------------
        stop : list
            a list that holds a single element that represents a flag
            to notify the program to kill the thread. we cannot pass an immutable
            primitive object which we can change by reference
        """
        while True:
            esc = sys.stdin.readline().strip()
            sys.stdout.flush()
            if esc == 'n':
                print ("-------\n SAFELY ENDING \n --------")
                stop[0] = True
                return



    def runLidar_client(self):
        """ Spins the lidar, parses and stores the readings.
        If jetson sends a list of bounding boxes that are detected,
        the program temporarily stops the lidar from scanning so that
        it can access the cache
        *** We use a condition_variable to make this main thread sleep
        until we find the depth in our |lidar_readings| dictionary
        """

        lidar = RPLidar('/dev/ttyUSB0')
        startTime = time.time()
        stopMessage = [False]
        stopReceiever = Thread(target = self.stopLidarThread, args=(stopMessage,))
        stopReceiever.start()
        f= open("scan_reading.txt","w+")
        for i, scan in enumerate(lidar.iter_scans(max_buf_meas=5000)):
            f.write("%s\n" % (scan))

            while not self._jetsonMsgCv.isSet():
                self._jetsonMsgCv.wait()
            if stopMessage[0] or i == c.MAX_NUM_ROTATION:
                break
            # unixTimeStamp made here. Round to the nearest second
            currentTime = round(time.time())
            self.parseLidarData(scan, currentTime)

        lidar.stop()
        f.close()
        lidar.stop_motor()
        lidar.disconnect()


    def computeLidarAngle(self, centerX):

        """ Computes the angle orientation of our bounding box
        based on the center X coordinate.

                    Field of view: 1280
        ------------------------------------------
        \                62.2 degrees          /
            \                |             /
                \            |          /
                    \        |       /
                        \    |    /
                            \| /


        Algorithm:
        --------------
        X = 1280 / 2
        if x-coordinate of the centroid < X:
            X_1 = X - x-coordinate
            theta = arctan( tan_31 * X_1 / X)
            return 360.0 - theta
        else:
            X_1 = x_coordinate - X
            theta = arctan( tan_31 * X_1 / X)
            return theta


        parameters:
        --------------------
        centerX : float
            the X-coordinate of the center of the bounding box
        """
        tan_31 = 0.60323856674
        pivotX = c.HORIZONTAL_RANGE/2
        sideFlag = c.LEFT_FOV if centerX < pivotX else c.RIGHT_FOV
        trueX = pivotX - centerX if (sideFlag == c.LEFT_FOV) else centerX - pivotX
        theta = math.degrees(math.atan ( c.TAN_31 * trueX / pivotX))
        angle = 360.0 - theta if (sideFlag == c.LEFT_FOV) else theta
        roundedAngle = round(angle * c.ROUND_DECIMAL_ANGLE)/c.ROUND_DECIMAL_ANGLE
        return roundedAngle





    def findDepthFromAngle(self, computedAngle, lidarDataEntry):
        """ Use the computed angle to find the closest angle measurement.
        |lidarDataEntry| is the lidar scan readings
        recorded for the corresponding time, and depth of all the angle
        at the time of interest is recorded here.

        algorithm:
        --------------------
        1. Iterate through the {angle : depth} key-value dict and
           keep track of the closest distance
        2. Keep track of any lidar readings whose angle was within
           +/- 1 degrees of the angle of interest
        3. Find the lidar reading of the angle, closest to the
           angle of interest

        parameters:
        --------------------
        computedAngle  : float
            Angle at which the object was located relative to the jetson
            camera.
        """
        minDiff = sys.maxsize
        recordedDepth = c.NOT_EXIST
        _ , _lidarReadings = lidarDataEntry
        for recordedAngle in _lidarReadings:
            angleDiff = abs(computedAngle - recordedAngle)
            if angleDiff < min(c.ANGLE_EPSILON, minDiff):
                minDiff = angleDiff
                recordedDepth = _lidarReadings[recordedAngle]
        return recordedDepth if recordedDepth != -1 else 50.0




    def parseJetsonRequest(self, message):
        """ processes the request from the jetson. Parses the
        coordinate values of the bounding boxes and the unixTimeStamp
        of these bounding boxes.


        jetson request format:
        ---------------------
        unixTimeStamp | numBoundingBoxes | [leftX, leftY, rightX ,rightY] * n


        parameters:
        --------------------
        message  : str
            Refer to the jetson request format above. The coordinates are
            the raw values returned by jetson
        """
        boundingBoxList = []
        payload = str(message.payload.decode("utf-8")).strip().split(";")
        requestedTime = float(payload[0])
        numBoundingBox = int(payload[1])
        tempArr = payload[2:-1]
        for i in range(0, numBoundingBox):
            leftX, leftY, rightX, rightY = list(map(float, tempArr[i].strip().split(",")))

            #With Tensorflow implementation
            # leftY, leftX, rightY, rightX = list(map(float, tempArr[i].strip().split(",")))
            # print (leftX, rightX, leftY, rightY)

            middleX = (leftX + rightX)/2
            boundingBoxList.append([leftX, leftY, rightX, rightY, middleX])
        return requestedTime, numBoundingBox, boundingBoxList



    def findClosestTime(self, requestedTime):
        """ finds the closest unix time that lidar has the reading on.
        Recall that the each element in | _lidarReading | is a dict that
        stores and updates lidar scans where each second represents
        a bin.

        algorithm:
        ------------------
        <Can be optimized>

        parameters :
        --------------------
        requestedTime : float
            the raw unixTime from jetson reading


        return :
        -------------------
        storedIndex : int
            the index at which the corresponding time is recorded in
            the | _lidarReading | list.
        """

        storedIndex, storedTime = None, c.NOT_EXIST
        minDiff = sys.maxsize
        for index in range (len(self._lidarReading)):
            lidarScanTime, _ = self._lidarReading[index]
            # print ("time to scan: ", lidarScanTime, requestedTime)
            timeDiff = abs(lidarScanTime - requestedTime)
            if timeDiff < min(c.TIME_EPSILON, minDiff):
                storedIndex = index
                storedTime = lidarScanTime
                minDiff = timeDiff
        return storedIndex, storedTime


    def _onConnect (self, _client, userdata, flags, rc):
        """ Callback for connecting to mosquitto
        """

        if rc==0:
            print("connected OK Returned code=",rc)
        else:
            print("Bad connection Returned code=",rc)

    def findDepthForBox(self, boundingBoxList, requestedTime):
        """ a wrapper function that iterates through all the bounding
        boxes detected from the jetson, computes the angle readings
        based on the coordinates of each, and find the depth reading.

        parameters:
        ---------------
        boundingBoxList : nested list
            all four coordinate values of each bounding box

        return :
        ---------------
        depthList : list
            list of floats that represent the depth of each bounding box
        """

        depthList = []
        self._jetsonMsgCv.clear()
        for (leftX, leftY, rightX, rightY, middleX) in boundingBoxList:
            estimatedTimeIndex, _ = self.findClosestTime(requestedTime)
            depth = c.NOT_EXIST
            computedAngleList = [self.computeLidarAngle(coordinate) for coordinate in [(leftX + middleX*c.CENTER_WEIGHT*2)/(c.CENTER_WEIGHT*2+1), (leftX + middleX*c.CENTER_WEIGHT)/(c.CENTER_WEIGHT+1), middleX, (rightX + middleX*c.CENTER_WEIGHT)/(c.CENTER_WEIGHT+1), (rightX + middleX*c.CENTER_WEIGHT*2)/(c.CENTER_WEIGHT*2+1)]]
            print ('center angle: ', computedAngleList[1])
            angleList = [self.findDepthFromAngle(angle, self._lidarReading[estimatedTimeIndex]) for angle in computedAngleList]
            validDepthList = [value for value in angleList if value != 50.0]
            depth = 50.0 if len(validDepthList) == 0 else sum(validDepthList)/len(validDepthList)
            depthList.append(depth)
        self._jetsonMsgCv.set()
        return depthList

    def publishDepth(self, depthListForBox, requestedTime):
        """ After finding the depths of each bounding box,
        the depthList is sent back as response to the jetson

        response format:
        ------------------
        | requestedTime | [ depths ]

        parameters:
        --------------------
        requestedTime : float
            the same raw unixTime received from jetson reading

        depthListForBox :
        """

        payload = '{:f};{};'.format(requestedTime , len(depthListForBox))
        for i in range(len(depthListForBox)):
            payload += '{:.1f};'.format(depthListForBox[i])
        self._client.publish(self._pubChannel, payload)
        print ('Response payload: ', payload, '\n')


    def _onMessage(self, _client, userData, message):
        """ Callback for receviing a message from jetson.
        An API given by the 'mosquitto' library
        """
        print(str(message.payload.decode("utf-8")).strip().split(";"))
        requestedTime, numBoundingBox, boundingBoxList = self.parseJetsonRequest(message)
        depthListForBox = self.findDepthForBox(boundingBoxList, requestedTime)
        self.publishDepth(depthListForBox, requestedTime)



    def run (self):
        """ Subscribes to a mosquitto broker to receive
        message from jetson, and starts lidar scan and processes its scans
        and stores them in |_lidarReading|
        """

        print ("===== START MQTT TEST ======")
        hostName = "192.168.1.107" #raspberry bi
        #hostName = "192.168.1.185" #localhost address
        port = 1883
        clientId = "jaewoo"
        self._client = mqtt.Client(client_id=clientId)

        try:
            self._client.on_connect=self._onConnect
            self._client.on_message=self._onMessage
            print ("===== CONNECT TO HOST ======")
            self._client.connect(hostName, port=port)
            self._client.loop_start()

            print ("===== SUBSCRIBE TO CHANNEL ======\n")
            self._client.subscribe(self._subChannel)
        except Exception as e:
            print(e)

        print ("==== START LIDAR ======")
        self.runLidar_client()

        #mqtt_thread.join()
        # sanity_check()
        self._client.loop_stop()


def main():
    lidar = Lidar()
    lidar.run()

if __name__=="__main__":
    main()
