import time
import random
from robot_visit import architecture_name_mapper as anm
from robot_visit.srv import Move
import math
import time
import rospy
import actionlib
from move_base_msgs.msg import *
from actionlib_msgs.msg import *
from nav_msgs.msg import Odometry
from control_msgs.msg import JointControllerState
from std_msgs.msg import Float64
import math
#
#
## @package robot_visit 
# \file move.py
# \brief This is the /move module, which is used for control the phscical movement of the robot.
# \author Zhouyang Hong 
# \version 0.1 
# \date 22/11/2022 
# 
# \details 

# Node: /move
# Service: /motion/move
# [None] 
# Description: 
# This node provide a service to state-decision-maker, when robot want to go somewhere, this service will be called to move 
# the robot to the destination.
#!/usr/bin/env python3

class Move_manager(object):
    """
    A class used to physicially move the robot
    ...
    Attributes ---------- 
    
    angle: float
    The current angle of joint1(rotation of cameara) 

    Methods -------
    execute_callback():
    This function will be called when required to move to target location.
    Here is just simply simulating this process by simply randomly wait an amount of time.

    Methods -------
    room_scan():
    This funtion will publish the desired angle of the cameara to move the robot to a desired angle.
    Methods -------

    mov_goal():
    This funtion will call action_server of move_base to move the robot to a target position.

    """
    def __init__(self):

        self.s = rospy.Service(anm.SVR_MOVE, Move,self.execute_callback)
        #self.odometry_sub = rospy.Subscriber("/odom",Odometry,self.odometry_callback)
       # self.pub_goal = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=1, latch=True)

        self.joint1_pose_pub = rospy.Publisher('/myRob1/joint1_position_controller/command', Float64, queue_size=10)
        self.angel_sub = rospy.Subscriber("/myRob1/joint1_position_controller/state", JointControllerState, self.callback_scan)
        
        # self.x_now = -100
        # self.y_now = -100
        # self.rotate_angle = 0
        self.angle = 0
    def room_scan(self):
        """"
        This function controls the commands passed to the joints. Joint 1 will rotate 360 degrees to scan the room."
        """
        # Create a publisher for the cmd_vel topic
        print("Scanning..")
        

        while self.angle < 2.9:
            try:
                self.joint1_pose_pub.publish(3)
                time.sleep(1)
            except:
                break
        while self.angle > 2.4:
            try:
                self.joint1_pose_pub.publish(-2.5)
                time.sleep(1)
            except:
                break
        self.joint1_pose_pub.publish(0)
        
        
    def callback_scan(self,msg):
        self.angle = msg.process_value
        
    def mov_goal(self,x,y):  #reach the desired goal inputed by user.
        
        print('going to: ', x, y)
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction) #start movebaseaction
        print('connected')
        client.wait_for_server() #waiting for response
        #create the message
        goal = MoveBaseGoal()
        #set goal parameter
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.orientation.w = 1
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        #send the goal
        client.send_goal(goal)
        #wait for result
        wait = client.wait_for_result(timeout=rospy.Duration(150))
        if not wait:
            #target not reached, calling cancle goal and return
            print("goal not reach")
            client.cancel_goal()
            return False
        print('goal_arrived')

        ################################Rotate()
        self.room_scan()

        return True #if reahced the target
            

    # def odometry_callback(self,odometry_msg):
    #     self.x_now = odometry_msg.pose.pose.position.x
    #     self.y_now = odometry_msg.pose.pose.position.y
        

    def execute_callback(self,msg):
        rospy.loginfo('recv_moving')
        # here, call move_target. but the robot shoul be able to know the name--location pair, can be done by asking the service.
        return self.mov_goal(msg.x,msg.y)

if __name__ == '__main__':
    # Initialise the node 
    rospy.init_node(anm.NODE_MOVE, log_level=rospy.INFO)
    server = Move_manager()
    rospy.spin()
