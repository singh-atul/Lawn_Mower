#!/usr/bin/env python
import rospy ,rospkg
import math
import serial
from std_msgs.msg import Float64 ,String
import time
from lawn_mower.srv import *


print 'In file'

flag = False
angle = None
s1=None
# arduino = serial.Serial('/dev/ttyUSB1', 9600)
# time.sleep(2)
# arduino.flushInput()

def client_srv_control_cmd(cmd_srv):
    print "----in client srv control cmd ------"
    rospy.wait_for_service('controller_cmd')
    try :
        controller_cmd = rospy.ServiceProxy('controller_cmd',controllerCMD)
        response_= controller_cmd(cmd_srv)
        return response_.cmd_response
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e



def getHeading(data):
    global flag
    global angle
    if flag:
        angle = data.data
        print 'Angle',angle
        flag = False


rospy.init_node("controller")


sub =  rospy.Subscriber('/compass_angle', Float64,getHeading)
flag = True
while flag:
    pass
s1=angle
print 'Current Angle is: ',angle,' degree'



# def sendData(data):
#     arduino.write(data)

# while True:
#     arduino.flushInput()
#     sendData(b'22,L')
#     time.sleep(.5)
#     d = arduino.readline()
#     if len(d)> 0 :
#         print(d)
#         break






# flag = True
# while flag:
#     pass
# print 'New Angle is: ',angle,' degree'
# print 'Diference: ', angle - s1



# 1s 4degree





# def sendData(data): 
#     arduino.write(data)

def move(action_data): 

    serv_response = client_srv_control_cmd(action_data) 
    print 'Got response from service as ' , serv_response
        
# def calculate_heading():
#     global flag
#     global angle
#     flag = True
#     while flag:
#         pass
#     return angle
    




performed_in_place_flag = False
goal = 253
heading_threshold = 3
diff = goal - angle
count = 1
print 'Goal',goal





while True: #m
    
    print 'Iteration ',count
    if not performed_in_place_flag:
        if diff < -180:
            diff += 360
        if diff > 180 :
            diff -= 360

    #time_to_move = abs(diff)/5    #5 when using inplace rotation on floor 10 when using rotation with motion
    time_to_move = abs(diff)*0.009
    print 'Time required to move will be ',time_to_move,' sec'
    if abs(diff) <= heading_threshold:
        print 'Desired Direction reached Now moving in straight direction'
        move((str(40)+',S').encode())
        break
    elif abs(diff)>20:
        print 'Performing Inplace rotation....'
        while abs(diff) >20:
            time_to_move_for_rotation = abs(diff)*0.009
            print 'Inplace rotation',time_to_move_for_rotation
            if diff < 0:
                move((str(time_to_move_for_rotation)+',L').encode())
            elif diff > 0:
                move((str(time_to_move_for_rotation)+',R').encode())
            flag = True
            while flag:
                pass
            diff = goal - angle
            if diff < -180:
                diff += 360
            if diff > 180 :
                diff -= 360
            performed_in_place_flag= True
        print 'Done with inplace rotation'
    elif diff < 0:
        print 'Left'
        move((str(time_to_move)+',l').encode())
    elif diff > 0:
        print 'Right'
        move((str(time_to_move)+',r').encode())

    flag = True
    while flag:
        pass
    diff = goal - angle

    print 'Difference left after itteration -',count,diff , ' goal - ' , goal , ' curr angle ' , angle
    count = count+1




# arduino.close()
