#!/usr/bin/env python

import random
import rospy
from std_msgs.msg import String
from robot_visit.msg import Charge, Control, Battery, MapBuilt
from robot_visit import architecture_name_mapper as anm
from armor_client import ArmorQueryClient
from armor_client import ArmorClient
from assignment2.srv import RoomInformation
import time
import datetime
from os.path import dirname, realpath
from nav_msgs.msg import Odometry
from control_msgs.msg import JointControllerState
from std_msgs.msg import Float64
import math

## @package robot_visit 
# \file build_map.py
# \brief This is the build_map module of ERL assignment robot_visit project and is used to build the topological map
# \author Zhouyang Hong 
# \version 0.1 
# \date 1/2/2023 
# 
# \details 
# Subscribes to: /Control 
# Node: /build_map

# Publishes to: 
# /state/map_built
# /myRob1/joint1_position_controller/command
# /myRob1/joint1_position_controller/state
# Service :
# [None] 
# 
# Description: 
# This node is used for building the topological map, it will 



class build_map:



    """
    A class used to scan around and build topological map for autonomous navigation of the robot.
    ...
    Attributes ---------- 

    map_built: bool
    When topological map is built this is set to true
   
    can_build: bool
    Set to true when markers are scanned

    markers: list
    Store total markers that have been received.
    
    angle: float
    The current angle of joint1(rotation of cameara) 

    scanned: bool
    Set to true if finishes scanning
   
    Methods -------
    room_scan():
    This funtion will publish the desired angle of the cameara to move the robot to a desired angle.

    Methods -------
    build():
    This funtion will get information of rooms from /room_info then build topological map

    """

    def __init__(self):
        """
        The constructor of class. Initializes parameters and publisher/suscriber
        
 """
        rospy.init_node(anm.NODE_MAP, log_level=rospy.INFO)
        self.can_build = False
        self.map_built = False
        self.markers = []
        self.angle = 0
        self.scanned = False
        self.map_publisher = rospy.Publisher(anm.TOPIC_MAP, MapBuilt, queue_size=1, latch=True)
        self.charge_subscriber = rospy.Subscriber(anm.TOPIC_CONTROL,Control,self.control_call_back)
        self.markers_subscriber = rospy.Subscriber(anm.TOPIC_MARKERS,String,self.markers_call_back)
        self.joint1_pose_pub = rospy.Publisher('/myRob1/joint1_position_controller/command', Float64, queue_size=10)
        self.angel_sub = rospy.Subscriber("/myRob1/joint1_position_controller/state", JointControllerState, self.callback_scan)
        self.path = dirname(realpath(__file__))
        self.path = self.path + "/"
        self.count_recv = 0
        self.started = False
        self.client = ArmorClient("client", "reference")
        self.client.utils.load_ref_from_file(self.path + "topological_map_abox.owl", "http://bnc/exp-rob-lab/2022-23", True, "PELLET", True, False)  # initializing with buffered manipulation and reasoning
        self.client.utils.mount_on_ref()
        self.client.utils.set_log_to_terminal(True)
        self.queryclient = ArmorQueryClient(self.client)
        

    def markers_call_back(self,msg):
        """
        The callback function of subscriber, when received markers, check if it is exists or add it, just used to receive markers.
 """
        
        result = msg.data.split(',')
        tem_str = []
        for iter in result:
            if len(iter) != 0:
                tem_str.append(iter)
            # if self.markers.find(iter) != -1:
            #     continue
            # self.markers.append(iter)
        self.markers = tem_str
        if self.scanned:
            self.scanned = False
            print('build markers')
            self.build()
            self.scanned = False
        #self.build() build should be called at here. to build map


    def control_call_back(self,msg):
        """
        The callback function of subscriber, when required to go, it means now robot can begin to build the map.
 """    
        if msg.cmd == 'go' and not self.started:
            self.started = True
            self.can_build = True
            self.room_scan()
            
        #here should be rotating the robot, so that to publish images

    def room_scan(self):
        """"
        This function controls the commands passed to the joints. Joint 1 will rotate 360 degrees to scan the room."
        """
        # Create a publisher for the cmd_vel topic
        print("Scanning..")
        self.joint1_pose_pub.publish(3)
        while self.angle < 3:
            try:
                time.sleep(1)
                self.joint1_pose_pub.publish(3)
                print("Scanning..1")
            except:
                break
        while self.angle > -2.4:
            try:
                time.sleep(1)
                self.joint1_pose_pub.publish(-2.5)
                print("Scanning..2")
            except:
                break
        self.scanned = True
        self.joint1_pose_pub.publish(0)
        

    def callback_scan(self,msg):
        """
        A callback for knowing the angle of camera right now.
 """
        self.count_recv += 1
        
        self.angle = msg.process_value
        if self.count_recv % 100 == 0:
            print('current_angle is ',self.angle)
    
    def build(self):
        """
        When markers are received, and scanning finishes, this function will be called to build topological map.
 """

        print('entered')
        rospy.wait_for_service('/room_info')
        rospy.wait_for_service('/armor_interface_srv')
        req_room_info = rospy.ServiceProxy('room_info',RoomInformation)
        curren_pos_visit = int(datetime.datetime.now().timestamp())

        ##################################################################
        # ask_for information
        for iter in self.markers:
            #call service
            resp = req_room_info(int(iter))
            print('room:'+resp.room)
            print('x,y: %d %d',resp.x,resp.y)
            print('connections:')
            if(resp.room == 'E'):
                self.client.manipulation.add_dataprop_to_ind('visitedAt', resp.room, "Long", '9999999999')
            else:
                self.client.manipulation.add_dataprop_to_ind('visitedAt', resp.room, "Long", str(curren_pos_visit))
            self.client.manipulation.add_dataprop_to_ind('pos_X', resp.room, 'float', str(resp.x))
            self.client.manipulation.add_dataprop_to_ind('pos_Y', resp.room, 'float', str(resp.y))
            
            for it in resp.connections:
                self.client.manipulation.add_objectprop_to_ind('hasDoor', resp.room, it.through_door)


        #add the information of robot.
        self.client.manipulation.add_objectprop_to_ind('isIn', 'Robot1', 'E')
        self.client.manipulation.add_dataprop_to_ind('now', 'Robot1', 'Long', str(curren_pos_visit))
        self.client.manipulation.add_dataprop_to_ind('urgencyThreshold', 'Robot1', 'Long', '400')  
                # modify

        if self.client.utils.apply_buffered_changes():
            pass#rospy.loginfo('consistent')
        else: 
            rospy.loginfo('apply_buffered_changes error')
        
        if self.client.utils.sync_buffered_reasoner():
            pass#rospy.loginfo('reasoned')
        else: 
            rospy.loginfo('reason error')
            self.client.armor_exceptions.ArmorServiceInternalError()

        self.client.utils.save_ref_with_inferences(self.path + "topological_map_abox_new.owl")
        self.client.utils.unmount_from_ref()
        map_msg = MapBuilt()
        map_msg.stamp = rospy.Time.now()
        map_msg.map_built = True
        self.map_publisher.publish(map_msg)


if __name__ == '__main__':
    # Get parameter and initialise this node as well as its publisher.
    rospy.wait_for_service('/room_info')
    rospy.wait_for_service('/armor_interface_srv')
    time.sleep(3)
    BT = build_map()
    rospy.spin()
    
        



