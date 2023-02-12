#!/usr/bin/env python

import random
import rospy
# Import constant name defined to structure the architecture.
from robot_visit import architecture_name_mapper as anm
# Import the message type (and sub-type) to be published.
from robot_visit.msg import Control

#
#
## @package robot_visit 
# \file control.py
# \brief This is the control module of ERL assignment robot_visit project 
# \author Zhouyang Hong 
# \version 0.1 
# \date 22/11/2022 
# 
# \details 

# Publishes to: /control

# Service :
# [None] 
# 
# Description: 
# This node is used to control the motion of the robot by simply inputting 'go' or 'stop'
# Inside, the publisher to topic /control will be created and wait for a command to be inputted.


if __name__ == '__main__':
    """
    A node that only consists of a single function
    This node is used to control the motion of the robot by simply inputting 'go' or 'stop'
    Inside, the publisher to topic /control will be created and wait for a command to be inputted.
    """

    # Get parameter and initialise this node as well as its publisher.
    rospy.init_node(anm.NODE_CONTROL, log_level=rospy.INFO)
    publisher = rospy.Publisher(anm.TOPIC_CONTROL, Control, queue_size=1, latch=True)
    msg = Control()
    
    while True:
        try:
            x = input('cmd: go or stop: ')
            if x == 'go':
                msg.stamp = rospy.Time.now()
                msg.cmd = 'go'
                publisher.publish(msg)
                
            if x == 'stop':
                msg.stamp = rospy.Time.now()
                msg.cmd = 'stop'
                publisher.publish(msg)
            if x == 'q':
                break
            
        except:
            break
    

