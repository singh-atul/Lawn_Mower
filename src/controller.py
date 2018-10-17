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

class controller:

    def __init__(self):
        self.tgt_lat = -1
        self.tgt_long =-1
        self.curr_lat = -1
        self.curr_long = -1

        #Compass
        self.heading = 0.0
        self.DEC_ANGLE = 0.2225

        self.WAYPOINT_DIST_TOLERANE = 0.10 # value in meters
        self.HEADING_TOLERANCE = 2 # value in degree
        self.TWO_PI = 6.2831855
        self.NUMBER_WAYPOINTS = 5 # static value , this describes number to waypoints to be combined for processing
        self.WAYPOINT_LIST = [] # value to be populated from the file 
        self.distanceToTarget = -1
        self.originalDistanceToTarget= -1
        self.targetHeading = 210
        self.currentHeading = -1
        self.turnDirection = "STOP"
        #populate waypoints to list
        self.getWayPoints()
        self.compassInteruppt = False
        self.WAYPOINT_INDEX = -1
        self.usingInteruppt = False
        # subscribe to RTK-GPS topic /vo
        #self.odom_sub = rospy.Subscriber('/vo', Odometry, self.sendCL)
        self.odom_sub = rospy.Subscriber('/fix', NavSatFix, self.sendCL)
        self.sub =  rospy.Subscriber('/compass_angle', Float64, self.getHeading)
        self.headingError = None
	    #Roatate till it does not have correct heading
        self.rotationFlag = False
        self.stop=False
        
    
        


    # get current lat , long and orientation
    def sendCL(self,data):
        if self.usingInteruppt : 
            print "******--- Fetching : GPS Data ---****** \n"
            '''
            self.curr_lat = data.pose.pose.position.x
            self.curr_long = data.pose.pose.position.y
            '''
            self.curr_lat = data.latitude
            self.curr_long = data.longitude
            self.usingInteruppt = False
    
    # call service to fetch next waypoint / read through file and store in list
    def getWayPoints(self):
        abs_path = rospack.get_path("lawn_mower") + "/script/ServerRequestHandler/" + "saved_points_sim.txt"
        with open(abs_path,'r') as pts:
		data = pts.readlines() 
        print data
        for i in data:
            #x,y = i.split(';')
            co = ast.literal_eval(i)
            #self.WAYPOINT_LIST.append([float(x),float(y)])
            self.WAYPOINT_LIST.append([co[0],co[1]])

    def getNextWayPoints(self):
        print '----  getNextWayPoints --------------'
        self.WAYPOINT_INDEX = self.WAYPOINT_INDEX + 1

        if (self.WAYPOINT_INDEX >= len(self.WAYPOINT_LIST) ):
            print("controller.py : Info : Processing last waypoint")
            rospy.signal_shutdown("Node Completed")
            return
            # while (1) :
            #     pass

        self.tgt_lat = self.WAYPOINT_LIST[self.WAYPOINT_INDEX][0]
        self.tgt_long= self.WAYPOINT_LIST[self.WAYPOINT_INDEX][1]

        

        self.processGPS() 
        print 'Waiting for GPS Signal and Compass Reading..'
        while self.usingInteruppt or self.compassInteruppt:
            pass
        print ' GPS Signal and Compass Reading Recieved..'
        self.distanceToTarget =  self.distanceToWaypoint()
        self.courseToWaypoint()       #--------------------------------------------------------
        print '***************************************'

    def processGPS(self):

        self.usingInteruppt = True
        self.compassInteruppt = True 
	
    def distanceToWaypoint(self):
        print ' ---------- DistanceToWaypoint -------------'
        delta = math.radians(self.curr_long - self.tgt_long)
        sdlong = math.sin(delta)
        cdlong = math.cos(delta)
        lat1 = math.radians(self.curr_lat)
        lat2 = math.radians(self.tgt_lat)
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
        '''
        # check to see if we have reached the current waypoint
        if (distanceToTarget <= self.WAYPOINT_DIST_TOLERANE) :
            print 'distance to target ', distanceToTarget
            self.getNextWayPoints()
     	'''
        print '***************************************'
        return distanceToTarget

    def courseToWaypoint(self):
        print '--------  CourseToWaypoint Function ---------'
        
        dlon = math.radians(self.tgt_long-self.curr_long)
        cLat = math.radians(self.curr_lat)
        
        tLat = math.radians(self.tgt_lat)
        a1 = math.sin(dlon) * math.cos(tLat)
        a2 = math.sin(cLat) * math.cos(tLat) * math.cos(dlon)
        a2 = math.cos(cLat) * math.sin(tLat) - a2
        a2 = math.atan2(a1, a2)
        
        self.targetHeading = math.degrees(a2)
        print 'before tRGET ' , self.targetHeading 
        if self.targetHeading < 0:
            self.targetHeading += 360
        # if self.targetHeading > 180 :
        #     self.targetHeading -= 360

        print '***************************************'
        

# read compass

    def getHeading(self,data):
        if self.compassInteruppt:
            self.heading = data.data
            print "####  controller.py : getHeading : Heading Data ",self.heading 
            self.compassInteruppt = False
        
    def readCompass(self):
        print "Current Compass Heading is ", self.heading , " degree"
        return self.heading


    def calculate_diff_angle(self):
        self.headingError = self.targetHeading - self.currentHeading
# service for sending command to aurdino through rasberry pi

    def client_srv_control_cmd(self,cmd_srv):
        print "----in client srv control cmd ------"
        rospy.wait_for_service('controller_cmd')
        try :
            controller_cmd = rospy.ServiceProxy('controller_cmd',controllerCMD)
            response_= controller_cmd(cmd_srv)
            return response_.cmd_response
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def move(self,action_data): 
        serv_response = self.client_srv_control_cmd(action_data) 
        print 'Got response from service as ' , serv_response



def loop():

    rospy.init_node("controller")
    #rospy.Subscriber("/stopMower",String,stopMower)
    
    
    while not rospy.is_shutdown() and not agv_controller.stop :
       
        agv_controller.getNextWayPoints()
        print '############################################################'
        print 'Distance to waypoint ',agv_controller.distanceToTarget
        print 'Current Latitude',agv_controller.curr_lat
        print 'Current Longitude',agv_controller.curr_long
        print 'Target Latitude',agv_controller.tgt_lat
        print 'Target Longitude',agv_controller.tgt_long

        agv_controller.currentHeading = agv_controller.readCompass()

        print '------ Current Heading : ',agv_controller.currentHeading,' ------------'
        print '------ Target Heading : ',agv_controller.targetHeading,' ------------'
        


        agv_controller.calculate_diff_angle()

        
        #print 'Desired Turn Mower is going to take: ',agv_controller.turnDirection
        print 'Going to perform action ....'
        count = 1 
        performed_in_place_flag = False
        dist = agv_controller.distanceToTarget
        while True:
            print 'Iteration ',count
            if not performed_in_place_flag:
                if  agv_controller.headingError< -180:
                    agv_controller.headingError += 360
                if agv_controller.headingError > 180 :
                    agv_controller.headingError -= 360
            time_to_move = abs(agv_controller.headingError)*0.17
            print 'Time required to move will be ',time_to_move,' sec'
            if dist < 0.5:
                print 'Goal Reached'
                break
            elif abs(agv_controller.headingError) <= agv_controller.HEADING_TOLERANCE:
                print 'Desired Direction reached Now moving in straight direction'
                agv_controller.move((str(30)+',F').encode())
                while dist>0.5:
                    #Update Current Location
                    agv_controller.usingInteruppt = True
                    while agv_controller.usingInteruppt :
                        pass
                    dist = agv_controller.distanceToWaypoint()
                agv_controller.move((str(30)+',S').encode())
            elif abs(agv_controller.headingError)>20:
                print 'Performing Inplace rotation....'
                while abs(agv_controller.headingError) >20:
                    time_to_move_for_rotation = abs(agv_controller.headingError)*0.009
                    print 'Inplace rotation',time_to_move_for_rotation
                    if agv_controller.headingError < 0:
                        agv_controller.move((str(time_to_move_for_rotation)+',L').encode())
                    elif agv_controller.headingError > 0:
                        agv_controller.move((str(time_to_move_for_rotation)+',R').encode())
                    agv_controller.currentHeading = agv_controller.readCompass()
                    agv_controller.calculate_diff_angle()
                    if agv_controller.headingError < -180:
                        agv_controller.headingError += 360
                    if agv_controller.headingError > 180 :
                        agv_controller.headingError -= 360
                    performed_in_place_flag= True
                print 'Done with inplace rotation'
            elif agv_controller.headingError < 0:
                print 'Left'
                agv_controller.move((str(time_to_move)+',l').encode())
            elif agv_controller.headingError > 0:
                print 'Right'
                agv_controller.move((str(time_to_move)+',r').encode())

            agv_controller.currentHeading = agv_controller.readCompass()
            agv_controller.calculate_diff_angle()

            print 'Difference left after itteration -',count,agv_controller.headingError , ' goal - ' , agv_controller.targetHeading , ' curr angle ' , agv_controller.currentHeading
            count = count+1
    print '###############################################################'

    

    print ("*******************Killing Node*****************************")
    rospy.signal_shutdown("lawn mower command STOP being called")


# call this function
agv_controller = controller()
loop()




