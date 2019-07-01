

from rplidar import RPLidar
from concurrent.futures import ThreadPoolExecutor
from threading import Thread, Lock, current_thread
from collections import deque
import time 
import sys
import paho.mqtt.client as mqtt
import os
import copy

"""
CONSTANT for lidar program
"""

RECORD_CONSTANT = 200
ROUND_DECIMAL_ANGLE = 2
ROUND_DECIMAL_WHOLE = 1
RASPI_CAM_HORIZ_FOV_ANGLE = 62.2 
MAX_INT = sys.maxsize
INIT_TIME = -1
NOT_EXIST = -1
TIME_EPSILON = 0.5
ANGLE_EPSILON = 1
"""
Deque data structure is thread-safe. 
Lock is unused
"""
lidar_data_deque = deque([(INIT_TIME, {}) for i in range(RECORD_CONSTANT)])
mqtt_process_lock = Lock()



"""
MQTT Specification
"""
host_name = "192.168.1.21" #jetson
host_name = "192.168.1.154" #raspberry address
#host_name = "192.168.1.182" #localhost address
port = 1883
client_id = "jjang_test"
sub_channel = "jetson"
pub_channel = "intercept"
client = None

def round_off(number): 
    return (round(number[1]* ROUND_DECIMAL_ANGLE)/ROUND_DECIMAL_ANGLE, number[2])


def organize_file(scan, measured_time, iteration): 
    start_time = time.time()
    print("Task Executed {}".format(current_thread()))
    index = iteration % RECORD_CONSTANT
    round_time = round(measured_time, ROUND_DECIMAL_WHOLE)
    
    lidar_data_list = map(round_off, scan)   
    lidar_data_map = dict(lidar_data_list)
    lidar_data_deque.append((round_time, lidar_data_map))
    lidar_data_deque.popleft()
    end_time = time.time() 
    print ("finished process of thread: ", end_time - start_time)
    #time.sleep(150)
    """
    Comment out the code below to check how many data points were preserved upon 
    storing the lidar data as a map. Compare the 'len' lidar_data_map to len of scan
    """
    
    #print (lidar_data_map)
    #print (len(lidar_data_map))


#earliest_index_translation = 0
#lidar_bank_index_lookup = {}


def run_lidar_client():
    executor = ThreadPoolExecutor(max_workers=10)    
    lidar = RPLidar('/dev/ttyUSB0')
    info = lidar.get_info()
    print(info)

    health = lidar.get_health()
    print(health)
    start_time = time.time()
    for i, scan in enumerate(lidar.iter_scans(max_buf_meas=5000)):
        #print('%d: Got %d measurments' % (i, len(scan)))
        
        current_time = round(time.time(), ROUND_DECIMAL_WHOLE)
       #print ('===========================')
        #print ('measurement at time: %f' % current_time)
        #print(scan, '\n')
        if i == 100000:
            break
        print ("Back to iteration: ", time.time() - start_time)
        executor.submit(organize_file, scan, current_time, i)
        start_time = time.time()
        
        #time.sleep(150)
    #print (lidar_data_bank)
    
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()


def sanity_check(): 
    print (lidar_data_deque)
    #for i in lidar_data_deque: 
    #    print (i[0])
    

def compute_lidar_angle(x_coordinate): 
    return 334.2

"""
Iterate to find the closest lidar reading
"""
def find_closest_time_index(given_time, lidar_data_list):  
    for index in range(0, len(lidar_data_list)): 
        time, _ = lidar_data_list[index]
        time_difference = abs(time - given_time)
        if time_difference  < TIME_EPSILON: 
            return index, time
            
    return None, NOT_EXIST


def find_dist_from_angle(computed_angle, lidar_data_tuple): 
    smallest_difference = MAX_INT
    recorded_depth = NOT_EXIST

    _, lidar_data = lidar_data_tuple
    for recorded_angle in lidar_data: 
        angle_difference = abs(computed_angle - recorded_angle)
        if angle_difference < ANGLE_EPSILON and angle_difference < smallest_difference: 
            smallest_difference = angle_difference 
            recorded_depth = lidar_data[recorded_angle]
            print ("ANGLE RECORDED FROM LIDAR ", recorded_angle)
    return recorded_depth

def on_message(client, userdata, message):
    payload = str(message.payload.decode("utf-8"))
    x_coordinate, given_time = map(float, payload.split(","))
    print("x-coordinate received:  " , x_coordinate)
    print("unix time recieved: ", given_time)
    copy_lidar_data = copy.deepcopy(lidar_data_deque)
    estimated_time_index, estimated_time  = find_closest_time_index(given_time, copy_lidar_data)
    depth = NOT_EXIST
    if estimated_time != NOT_EXIST: 
        computed_angle = compute_lidar_angle(x_coordinate)
        print ("ANGLE COMPUTED FROM X-COORD: ", computed_angle)
        print ("TIME RECORDED WITH LIDAR: ", estimated_time)
        depth = find_dist_from_angle(computed_angle, copy_lidar_data[estimated_time_index])
        print ("THIS IS DEPTH: ", depth)
    print("DONE CALLBACK")
    
    client.publish(pub_channel, depth)
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
def run_mqtt_client():
    client = mqtt.Client(client_id=client_id)
    # client.username_pw_set(username, password)

    client.on_connect=on_connect
    client.on_message=on_message
    print ("===== CONNECT TO HOST ======")
    client.connect(host_name, port=port)

    client.loop_start()
    print ("===== SUBSCRIBE TO CHANNEL ======")
    client.subscribe(sub_channel)

    # print ("===== PUBLISH TO HOST ======")

    

def main():
    print ("===== START MQTT TEST ======")
    mqtt_thread = Thread(target = run_mqtt_client)
    mqtt_thread.start()

    print ("==== START LIDAR ======")
    run_lidar_client()
    sanity_check()

if __name__=="__main__":
    main()




