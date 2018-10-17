#!/usr/bin/env python
import rospy ,rospkg
import math
# from geometry_msgs.msg import Twist
#from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
import numpy as np
from std_msgs.msg import Float64 ,String
import ast
from lawn_mower.srv import *
import time
import serial
rospack = rospkg.RosPack()


print 'In file'

flag = False
angle = None
s1=None
arduino = serial.Serial('/dev/ttyUSB2', 9600)
time.sleep(2)
arduino.flushInput()

def getHeading(data):
    global angle
    angle = data.data

rospy.init_node("controller")


curr_lat = None
curr_long = None

def sendCL(data):
    global curr_lat
    global curr_long
    curr_lat = data.latitude
    curr_long = data.longitude





sub =  rospy.Subscriber('/compass_angle', Float64,getHeading)
odom_sub = rospy.Subscriber('/fix', NavSatFix,sendCL)
time.sleep(0.1)
s1=angle
print 'Current Angle is: ',angle,' degree and its type is ',type(angle)
print 'Curent Latitude ', curr_lat
print 'Curent Longitude ', curr_long


def sendData(data):
    print 'Sending Data ..... '
    while True:
        arduino.flushInput()
        arduino.write(data)
        time.sleep(.5)
        d = arduino.readline()
        if len(d)> 0 :
            print(d)
            break
    print 'Comand Executing'




# # 1s 4degree





# # def sendData(data): 
# #     arduino.write(data)

# def move(action_data): 

#     serv_response = client_srv_control_cmd(action_data) 
#     print 'Got response from service as ' , serv_response
        
# # def calculate_heading():
# #     global flag
# #     global angle
# #     flag = True
# #     while flag:
# #         pass
# #     return angle
    



#DONE5.1 / 1
def wrap_angle(diff):
    if diff < -180:
        diff +=360
    
    if diff > 180:
        diff -=360
    return diff


#DONE5
def get_desired_heading(goal,heading_threshold = 3):
    global angle
    diff = goal - angle
    count = 1
    print 'Goal',goal

    while True: #m
        print 'Performing Iteration ',count
        print 'Before Wrapping',diff
        diff = wrap_angle(diff)
        if diff > 0 :
            goal -= 5
        else:
            goal += 5
        #Set offset to reach goal
        # if diff < 0 :
        #     goal -=4
        # else:
        #     goal +=4
        print 'After wrapping',diff
        if abs(diff) <= heading_threshold:
            print 'Desired direction reached '
            break
        else:
            while abs(diff) > heading_threshold:
                if diff < 0:
                    print 'Turning Left'
                    sendData((str(23)+',L').encode())
                else:
                    print 'Turning Right'
                    sendData((str(23)+',R').encode())
                
                diff = wrap_angle(goal - angle)
                print 'Difference after rotating is ',diff
            print 'Final Facing Angle Before Stopiing ',angle
            sendData((str(30)+',S').encode())
            print 'Final Facing Angle After Stopiing ',angle
    print 'print desired angle ',goal, 'degree reached'






#Done4
def distanceToWaypoint(tgt_lat,tgt_long,curr_lat,curr_long):
    delta = math.radians(curr_long - tgt_long)
    sdlong = math.sin(delta)
    cdlong = math.cos(delta)
    lat1 = math.radians(curr_lat)
    lat2 = math.radians(tgt_lat)
    slat1 = math.sin(lat1)
    clat1 = math.cos(lat1)
    slat2 = math.sin(lat2)
    clat2 = math.cos(lat2)
    delta = (clat1 * slat2) - (slat1 * clat2 * cdlong)
    delta = math.pow(delta,2)
    delta += math.pow(clat2 * sdlong,2)
    delta = math.sqrt(delta) 
    denom = (slat1 * slat2) + (clat1 * clat2 * cdlong)
    delta = math.atan2(delta, denom)
    distanceToTarget =  delta * 6372795     #In meters
    return distanceToTarget



#Done3
def course_to_waypoint(tgt_lat,tgt_long,curr_lat,curr_long):
    dlon = math.radians(tgt_long - curr_long)
    cLat = math.radians(curr_lat)
    tLat = math.radians(tgt_lat)
    a1 = math.sin(dlon) * math.cos(tLat)
    a2 = math.sin(cLat) * math.cos(tLat) * math.cos(dlon)
    a2 = math.cos(cLat) * math.sin(tLat) - a2
    a2 = math.atan2(a1, a2)
    targetHeading = math.degrees(a2)
    print 'before tRGET ' , targetHeading 
    if targetHeading < 0:
        targetHeading += 360
    print 'After ',targetHeading
    return targetHeading



#Done2
def get_next_waypoint(WAYPOINT_INDEX , WAYPOINT_LIST):
    print 'In waypoint Function'
    print 'Going to fetch waypoint from waypoint list at index, ',WAYPOINT_INDEX
    if WAYPOINT_INDEX>len(WAYPOINT_LIST):
        print 'Allpoints are traversed ..'
        sendData((str(30)+',S').encode())
    tgt_lat = WAYPOINT_LIST[WAYPOINT_INDEX][0]
    tgt_long= WAYPOINT_LIST[WAYPOINT_INDEX][1]
    return [tgt_lat,tgt_long]



#DONE 1
def getWayPoints():
    WAYPOINT_LIST= []
    abs_path = rospack.get_path("lawn_mower") + "/script/ServerRequestHandler/" + "saved_points_sim.txt"
    with open(abs_path,'r') as pts:
	data = pts.readlines() 
    print data
    for i in data:
        co = ast.literal_eval(i)
        WAYPOINT_LIST.append([co[0],co[1]])
    return WAYPOINT_LIST



#DONE6
def move_forward(tgt_lat,tgt_long,curr_lat,curr_long,goal):
    dist = distanceToWaypoint(tgt_lat,tgt_long,curr_lat,curr_long)
    dist_check = dist
    while dist>0.1:
        print 'Distance left to cover is ',dist
        if dist > 1:
            print 'meter : Moving at speed of 40'
            sendData((str(40)+',F').encode())
        if dist <= 1 and dist >= 0.3:
            print 'meter : Moving at speed of 30'
            sendData((str(30)+',F').encode())
        if dist < 0.3:
            print 'meter : Moving at speed of 20'
            sendData((str(20)+',F').encode())

        print 'Calculating the updated distance to move '
        dist = distanceToWaypoint(tgt_lat,tgt_long,curr_lat,curr_long)

        print 'Check if distance is decreasing'
        if dist_check< dist:
            sendData((str(30)+',S').encode())
            print 'FIXING THE ORIENTATION'
            get_desired_heading(goal)
            dist = distanceToWaypoint(tgt_lat,tgt_long,curr_lat,curr_long)
        else:
            dist_check = dist
    print 'Point reached '
    sendData((str(30)+',S').encode())



def pipeline():
    global curr_lat
    global curr_long
    global angle
    print 'Going to load datapoints in waypoints list ...'
    WAYPOINT_LIST = getWayPoints()
    print len(WAYPOINT_LIST),' point Loaded in waypoint '
    index = 0
    while True:
        print 'Fetching goal Coordinates' 
        [tgt_lat,tgt_long] = get_next_waypoint(index,WAYPOINT_LIST)
        print 'Goal Coordinates Fetched'
        #print 'Distance to waypoint ',distanceToTarget

        print 'Calling course to waypoint to get the course of waypoints'
        goal = course_to_waypoint(tgt_lat,tgt_long,curr_lat,curr_long)
        print 'Course to waypoint recieved is, ',goal

        print 'Calling function to get the distance between current and target coordinates'
        dist = distanceToWaypoint(tgt_lat,tgt_long,curr_lat,curr_long)
        print 'distance to waypoint calculated to be  ',dist
        # print ()
        print '------- SUMMARY ---------'
        print 'Current Latitude',curr_lat
        print 'Current Longitude',curr_long
        print 'Target Latitude',tgt_lat
        print 'Target Longitude',tgt_long
        print 'distance to waypoint ',dist
        print 'Course to waypoint recieved is, ',goal
        print '---------------------------'
        # print ()

        print '----ORIENTATION FIXING ------'
        print 'Fix the orientation by calling the get desired function'
        print 'Going in get_desired_function' , goal
        get_desired_heading(goal)
        print 'Out of get_desired function'
        print 'Difference in angle after running get desired function is ' ,wrap_angle(goal - angle)
        print ' -  - - - - - - - - - - - - - '

        print 'Calling function again to get the distance between current and target coordinates as it might have get changed while rotation'
        dist = distanceToWaypoint(tgt_lat,tgt_long,curr_lat,curr_long)
        print 'distance to waypoint calculated to be  ',dist



        print '------ MOVING -----'
        print 'Going to move forward till the time the point doesnt reaches the target'
        move_forward(tgt_lat,tgt_long,curr_lat,curr_long,goal) 
        print 'Out of move_forward function'
        print '- - - - - - - - - - -'
        index +=1
        print 'GOING FOR next point at index ',index , 'in waypoint list'


arduino.close()
