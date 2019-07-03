

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


def parse_data(scan, measured_time, iteration):  
    global current_second_unix_time
    global lidar_data_list
    global RECORD_CONSTANT
    global current_sec_lidar_reading
    
    if current_second_unix_time != None and abs(measured_time - current_second_unix_time) >= 0.5: 
        current_sec_lidar_reading = {}
    #Not floor function 
    current_second_unix_time = round(measured_time) 

    map_index = current_second_unix_time % RECORD_CONSTANT
    print (current_second_unix_time, measured_time)
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
        while not cv.isSet(): 
            print ("LOCKING")
            cv.wait() 
        current_time = round(time.time(), ROUND_DECIMAL_WHOLE)
       #print ('===========================')
        #print ('measurement at time: %f' % current_time)
        #print(scan, '\n')
        if i == 100000:
            break
        #print ("Back to iteration: ", time.time() - start_time)
        executor.submit(parse_data, scan, current_time, i)
        #start_time = time.time()
        
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
    print ("--------\n")
    
    for recorded_angle in lidar_data: 
        angle_difference = abs(computed_angle - recorded_angle)
        print ("computed/recorded angle: ", computed_angle, recorded_angle)
        if angle_difference < ANGLE_EPSILON and angle_difference < smallest_difference: 
            smallest_difference = angle_difference 
            recorded_depth = lidar_data[recorded_angle]
            print ("ANGLE RECORDED FROM LIDAR ", recorded_angle)
    print ("--------\n")

    return recorded_depth


def on_message(client, userdata, message):    
    payload = str(message.payload.decode("utf-8"))
    x_coordinate, given_time = map(float, payload.split(","))
    print("x-coordinate received:  " , x_coordinate)
    print("unix time recieved: ", given_time)
    global cv
    global lidar_data_list
    cv.clear()
    
    estimated_time_index, estimated_time  = find_closest_time_index(given_time, lidar_data_list)
    depth = NOT_EXIST
    #time.sleep(5)
    if estimated_time != NOT_EXIST: 
        computed_angle = compute_lidar_angle(x_coordinate)
        print ("ANGLE COMPUTED FROM X-COORD: ", computed_angle)
        print ("TIME RECORDED WITH LIDAR: ", estimated_time)
        depth = find_dist_from_angle(computed_angle, lidar_data_list[estimated_time_index])
    print("DONE CALLBACK")
    client.publish(pub_channel, depth)
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




