

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

"""
CONSTANT for lidar program
"""

RECORD_CONSTANT = 100
ROUND_DECIMAL_ANGLE = 2
ROUND_DECIMAL_WHOLE = 1
RASPI_CAM_HORIZ_FOV_ANGLE = 62.2
MAX_INT = sys.maxsize
INIT_TIME = -1
NOT_EXIST = -1
TIME_EPSILON = 1
ANGLE_EPSILON = 1
HORIZONTAL_RANGE = 1280
LEFT_FOV = 0
RIGHT_FOV = 1

"""
Deque data structure is thread-safe.
Lock is unused
"""
lidar_data_deque = deque([(INIT_TIME, {}) for i in range(RECORD_CONSTANT)])
lidar_data_lock = Lock()


"""
Optimized Implementaiton
"""
current_sec_lidar_reading = {}
current_second_unix_time = None
lidar_data_list = list([(INIT_TIME, {}) for i in range(RECORD_CONSTANT)])
cv = Event()
cv.set()


"""
MQTT Specification
"""
host_name = "192.168.1.165" #raspberry bi
#host_name = "192.168.1.185" #localhost address
port = 1883
client_id = "jjang_test"
sub_channel = "jetson"
pub_channel = "intercept"
client = None



def round_off(number):
    return (round(number[1]* ROUND_DECIMAL_ANGLE)/ROUND_DECIMAL_ANGLE, number[2])


def parse_data(scan, measured_time, iteration):
    global current_second_unix_time
    global lidar_data_list
    global RECORD_CONSTANT
    global current_sec_lidar_reading

    if current_second_unix_time != None and abs(measured_time - current_second_unix_time) >= 0.5:
        current_sec_lidar_reading = {}
    #Not floor function
    current_second_unix_time = measured_time

    map_index = current_second_unix_time % RECORD_CONSTANT
    #print (current_second_unix_time, measured_time)
    lidar_data_list[map_index] = (current_second_unix_time, current_sec_lidar_reading)
    lidar_data = map(round_off, scan)
    current_sec_lidar_reading.update(lidar_data)



    #print("Task Executed {}".format(current_thread()))
    #print ("finished process of thread: ", end_time - start_time)
    #time.sleep(150)
    """
    Comment out the code below to check how many data points were preserved upon
    storing the lidar data as a map. Compare the 'len' lidar_data_map to len of scan
    """

    #print (lidar_data_map)
    #print (len(lidar_data_map))


#earliest_index_translation = 0
#lidar_bank_index_lookup = {}


# buggy = [[i*0.1, i*0.1] for i in range(0, 100000)]
def stop_thread(stop):
    while True:
        esc = sys.stdin.readline().strip()
        sys.stdout.flush()
        if esc == 'n':
            print ("-------")
            print ('SAFELY ENDING')
            print ("-------")
            stop[0] = True
            return

def run_lidar_client():
    executor = ThreadPoolExecutor(max_workers=10)

    lidar = RPLidar('/dev/ttyUSB0')

    start_time = time.time()

    stop_message = [False]
    stop_receiver = Thread(target = stop_thread, args=(stop_message,))
    stop_receiver.start()

    for i, scan in enumerate(lidar.iter_scans(max_buf_meas=5000)):
    #for i, scan in enumerate(buggy):
        #print('%d: Got %d measurments' % (i, len(scan)))
        while not cv.isSet():
            print ("LOCKING")
            cv.wait()
        if stop_message[0]:
            break
        #current_time = round(time.time()*2)/2
        current_time = round(time.time())
       #print ('===========================')

        #print ('measurement at time: %f' % current_time)
        #print(scan, '\n')
        #esc = 1

        if i == 1000000:
            break
        #print ("Back to iteration: ", time.time() - start_time)
        executor.submit(parse_data, scan, current_time, i)
        #start_time = time.time()

        #time.sleep(150)
    #print (lidar_data_bank)
    stop_receiver.join(1)
    executor.shutdown()
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()


def sanity_check():
    print (lidar_data_deque)
    #for i in lidar_data_deque:
    #    print (i[0])


def compute_lidar_angle(x_coordinate):
    global HORIZONTAL_RANGE
    tan_31 = 0.60323856674
    X = HORIZONTAL_RANGE/2
    #left_FOV = 0 and right_FOV = 1
    side_flag = LEFT_FOV if x_coordinate < X else RIGHT_FOV
    X_1 = X - x_coordinate if (side_flag ==LEFT_FOV) else x_coordinate - X
    theta = math.degrees(math.atan ( tan_31 * X_1 / X))
    angle = 360.0 - theta if (side_flag == LEFT_FOV) else theta
    return angle

    """
    if x_coordinate < X:
        X_1 = X - x_coordinate
        theta = math.atan( tan_31 * X_1 / X)
        return 360.0 - theta
    else:
        X_1 = x_coordinate - X
        theta = math.atan( tan_31 * X_1 / X)
        return theta
    """
"""
Iterate to find the closest lidar reading
"""
def find_closest_time_index(given_time, lidar_data_list):
    global TIME_EPSILON
    global NOT_EXIST

    stored_index, stored_time = None, NOT_EXIST
    shortest_diff = sys.maxsize
    for index in range(0, len(lidar_data_list)):
        time, _ = lidar_data_list[index]
        time_difference = abs(time - given_time)
        if time_difference < 3:
            print ("time measure: ", given_time, time)
        if time_difference  < TIME_EPSILON and time_difference < shortest_diff :
            print ('FOUND TIME')
            stored_index = index
            stored_time = time
            shortest_time = time_difference

    return stored_index, stored_time



def find_dist_from_angle(computed_angle, lidar_data_tuple):
    smallest_difference = MAX_INT
    recorded_depth = NOT_EXIST
    _, lidar_data = lidar_data_tuple

    for recorded_angle in lidar_data:
        angle_difference = abs(computed_angle - recorded_angle)
        #print ("computed/recorded angle: ", computed_angle, recorded_angle)
        if angle_difference < ANGLE_EPSILON and angle_difference < smallest_difference:
            smallest_difference = angle_difference
            recorded_depth = lidar_data[recorded_angle]
            #print ("ANGLE RECORDED FROM LIDAR ", recorded_angle)

    return recorded_depth


def on_message(client, userdata, message):
    global cv
    global lidar_data_list

    print ('INCOMING MESSAGE')
    bounding_box_list = []
    payload = str(message.payload.decode("utf-8")).strip()
    print ("Payload format: ", payload.split(";"))
    arr = payload.split(";")[:-1]

    for i in range(0, len(arr)):
        left_x, left_y, right_x, right_y,  middle_x, given_time = map(float, arr[i].strip().split(","))
        bounding_box_list.append([left_x, left_y, right_x, right_y, middle_x, given_time])
        print("x-coordinate received:  " , middle_x)
        print("unix time recieved: ", given_time)



    cv.clear()
    depth_list = []
    for box in bounding_box_list:
        print ("debugging: ", box)
        left_x, left_y, right_x, right_y, middle_x, given_time = box
        estimated_time_index, estimated_time  = find_closest_time_index(given_time, lidar_data_list)
        depth = NOT_EXIST

        if estimated_time != NOT_EXIST:
            computed_angle = compute_lidar_angle(middle_x)
            print ("ANGLE COMPUTED FROM X-COORD: ", computed_angle)
            print ("TIME RECORDED WITH LIDAR: ", estimated_time)
            depth = find_dist_from_angle(computed_angle, lidar_data_list[estimated_time_index])
            depth_list.append(depth)
    """
        payload: number of boxes * [depth, left_x, left_y, right_x, right_y, unix_time ]
    """
    print ('NUMBER of boxes: ', len(bounding_box_list))
    payload = '{};'.format(len(bounding_box_list))
    for i in range(len(bounding_box_list)):
        payload += '{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f};'.format(depth_list[i], bounding_box_list[i][0], bounding_box_list[i][1], bounding_box_list[i][2], bounding_box_list[i][3], bounding_box_list[i][5])
    client.publish(pub_channel, payload)
    print ('what is sent back: ', payload)
    print ("Keep going\n")
    cv.set()


    """
    SEND back a response based on the depth of the cone
    """




#ON CONNECTION CALLBACK
def on_connect(client, userdata, flags, rc):
    if rc==0:
        print("connected OK Returned code=",rc)
    else:
        print("Bad connection Returned code=",rc)


#Publish a command to the "topic and state channel"
#def run_mqtt_client():


    # print ("===== PUBLISH TO HOST ======")




def main():
    print ("===== START MQTT TEST ======")


    #mqtt_thread = Thread(target = run_mqtt_client)
    #mqtt_thread.start()
    client = mqtt.Client(client_id=client_id)
        # client.username_pw_set(username, password)

    client.on_connect=on_connect
    client.on_message=on_message
    print ("===== CONNECT TO HOST ======")
    client.connect(host_name, port=port)

    client.loop_start()
    print ("===== SUBSCRIBE TO CHANNEL ======")
    client.subscribe(sub_channel)
    print ("\n")




    print ("==== START LIDAR ======")
    run_lidar_client()

    #mqtt_thread.join()
    # sanity_check()
    client.loop_stop()

if __name__=="__main__":
    main()
