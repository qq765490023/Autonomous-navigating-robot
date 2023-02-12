#!/usr/bin/env python

import random
import rospy
from robot_visit.msg import Charge, Control, Battery
from robot_visit import architecture_name_mapper as anm
import time
CAPACITY_ = 5000


## @package robot_visit 
# \file battery_control.py
# \brief This is the battery_control module of ERL assignment robot_visit project 
# \author Zhouyang Hong 
# \version 0.1 
# \date 22/11/2022 
# 
# \details 
# Subscribes to: /charge 
# Node: /battery 

# Publishes to: /state/battery

# Service :
# [None] 
# 
# Description: 
# This node is used for managing the battery, it will publish the current capacity of the battery to decision-maker
# to support decision-making. When robot needs charge and is in the right location, 
# here is what responsible for charging.



class battery:



    """
    A class used to know where the robot should go next,  move the robot, and also update topolical location

    ...
    Attributes ---------- 
    
    charging: bool
    When ruiquired by the /state_decision_maker node to charge, this value will be set to true, to inform run() to charge.
    
    moving: bool
    This is used to receive the command from /move. If control node says 'go', the robot will go even not fully charged.

    capacity: int
    Used to represent the current capacity of battery and will be published to onther node.

    
    Methods -------
    run():
    This funtion will publish the capacity of batttery to other node, if required to charge, charge action will executed.

    """

    def __init__(self):
        """
        The constructor of class. Initializes parameters and publisher/suscriber
        
 """
        rospy.init_node(anm.NODE_BATTERY, log_level=rospy.INFO)
        self.publisher = rospy.Publisher(anm.TOPIC_BATTERY, Battery, queue_size=1, latch=True)
        self.charge_subscriber = rospy.Subscriber(anm.TOPIC_CONTROL,Control,self.control_call_back)
        self.charge_subscriber = rospy.Subscriber(anm.TOPIC_CHARGE,Charge,self.charge_call_back)
        self.charging = False
        self.capacity = CAPACITY_
        self.battery_msg = Battery()
        self.moving = False
        
    # to avoid race and compete with controll command
    def charge_call_back(self,msg):
        """
        The call back function of subscriber, when required to charge, self.charging wil be set to true.
 """

        print('Charge request')
        if msg.charge:
            print('Charge confirm')
            self.charging = msg.charge

    # Force stop charging.
    def control_call_back(self,msg):
        """
        The callback function of subscriber, when required to move, self.moving wil be set to true.
 """

        if msg.cmd == 'go':
            self.charging = False
            self.moving = True
        if msg.cmd == 'stop':
            self.moving = False
    
    def run(self):
        """
        The core function of this class, This function will execute endlessly. When informed to charge, here will do the stuff 
        to charge the battery, and also simulate the consumption of battery.
 """
        
        time.sleep(1)
        while self.charging and self.capacity < CAPACITY_:

            try:
                time.sleep(1)
                self.capacity += 200

                if self.capacity > CAPACITY_:
                    self.capacity = CAPACITY_
                    self.battery_msg.stamp = rospy.Time.now()
                    self.battery_msg.capacity = self.capacity
                    self.publisher.publish(self.battery_msg)
                    rospy.loginfo('Battery OK: %d',self.capacity)
                    self.charging = False
                    break
                rospy.loginfo('Battery charging: %d',self.capacity)  
                
            except:
                break
        
        #Simulate Battery
        if self.capacity > 0 and self.moving:
            self.capacity -= 1
        self.battery_msg.stamp = rospy.Time.now()
        self.battery_msg.capacity = self.capacity
        self.publisher.publish(self.battery_msg)
        
        
    

if __name__ == '__main__':
    # Get parameter and initialise this node as well as its publisher.
    BT = battery()
    tem_time = 1
    while True:
        try:
            tem_time = tem_time + 1
            BT.run()
            if tem_time % 50 == 0:
                rospy.loginfo('Battery now: %d',BT.capacity)


        except:
            break
        # if robot is at E, capacity add 10, every 1 second.

        
