#! /usr/bin/env python

import random
import rospy
# Import constant name defined to structure the architecture.
from robot_visit import architecture_name_mapper as anm
# Import the ActionServer implementation used.
from actionlib import SimpleActionServer
# Import custom message, actions and services.
from robot_visit.srv import SetPose
import robot_visit  # This is required to pass the `PlanAction` type for instantiating the `SimpleActionServer`.
from robot_visit.msg import Control
import roslib
import smach
import smach_ros
import time
import datetime
from armor_client import ArmorQueryClient
from armor_client import ArmorClient
from os.path import dirname, realpath
from robot_visit.msg import Control, Battery, Charge, MapBuilt
from robot_visit.srv import Move
CHARGING_TIME = 5
DELAY_TIME = 2

#
#
## @package robot_visit 
# \file state_decision_maker.py
# \brief This is the state-decision-maker module of ERL assignment robot_visit project 
# \author Zhouyang Hong 
# \version 0.1 
# \date 22/11/2022 
# 
# \details 
# Node: /state_decision_maker 
# Subscribes to: state/battery
# Subscribes to: state/control
# Subscribes to: state/charge
# Client of service: move 

# Description: 

# state-decision-maker consists of state-machine and decision-maker those 2 parts. Conceptually they are different, the decision-maker is used as a parameter for the state-machine. 
# decision-maker can tell state-machine what decision to make for the next step, and also communicate with Move(which is to simulate the motion of robot), update topological locations. 
# state-machine will call the functions of  state-decision-maker.



class decision_maker():


    """
    A class used to know where the robot should go next,  move the robot to that position, and also update topolical location

    ...
    Attributes ---------- 
    
    onto_path : str
    Used to know the file location of Ontology file(not in use now). 

    battery_low: bool
    Used to know if robot has sufficient battery.

    go_work: bool
    Used to check if receiced the command from /control node

    battery_capacity: int
    Used to represent the current capacity of battery.

    track : str
    Used to show the history of locations that the robot visited.
    
    map_built: bool
    Used to know that the map is built.
    
    Methods -------
    
    update_pos(next_v):
    Update the current position [next_v] the robot to topological map.
    return true if succeeded


    move(next_v)
    Physically move the robot to location [next_v]
    return true if succeeded

    next_go():
    Returns the position the the robot should go next, and if no need wait(For urgent case).[location, no_wait]
    """


    def __init__(self):

        """
        The Constructor, initializes publisher, subscriber, and variables
 """
        
        self.Control_sub = rospy.Subscriber(anm.TOPIC_CONTROL,Control,self.control_callback)
        self.Battery_sub = rospy.Subscriber(anm.TOPIC_BATTERY,Battery,self.battery_callback)
        self.Map_sub = rospy.Subscriber(anm.TOPIC_MAP,MapBuilt,self.map_built_callback)
        
        self.Charge_pub = rospy.Publisher(anm.TOPIC_CHARGE, Charge, queue_size = 1, latch=True)

        self.battery_low = True
        self.go_work = False
        self.battery_capacity = 0
        self.track = 'E'
        self.map_built = False
        self.robot_at = ''
        ############################ test working
        # self.battery_capacity = 5000
        # self.go_work = True
        # tem_msg = MapBuilt()
        # tem_msg.map_built = True
        # self.map_built_callback(tem_msg)
        # self.map_built = True
        # self.battery_low = False
        ################################################
        # # get robot initial position
        # robot_at_tem = self.client.query.objectprop_b2_ind('isIn','Robot1')
        # if len(robot_at_tem) > 0:
        #     robot_at_tem[0] = robot_at_tem[0].replace('<','')
        #     robot_at_tem[0] = robot_at_tem[0].replace('>','')
        #     robot_at_tem[0] = robot_at_tem[0].replace('http://bnc/exp-rob-lab/2022-23#','')
        #     self.robot_at = robot_at_tem[0]
            
        # else:
        #     rospy.loginfo('get initial postion error')
    def map_built_callback(self,msg):
        """
        A callback for knowing the the topological map is built.
 """

        print('map_built_called')
        self.map_built = msg.map_built
        path = dirname(realpath(__file__))
        path = path + "/"
        self.client = ArmorClient("client", "reference")
        self.client.utils.load_ref_from_file(path + "topological_map_abox_new.owl", "http://bnc/exp-rob-lab/2022-23", True, "PELLET", True, False)  # initializing with buffered manipulation and reasoning
        self.client.utils.mount_on_ref()
        self.client.utils.set_log_to_terminal(True)
        self.queryclient = ArmorQueryClient(self.client)

        # get robot initial position
        robot_at_tem = self.client.query.objectprop_b2_ind('isIn','Robot1')
        if len(robot_at_tem) > 0:
            robot_at_tem[0] = robot_at_tem[0].replace('<','')
            robot_at_tem[0] = robot_at_tem[0].replace('>','')
            robot_at_tem[0] = robot_at_tem[0].replace('http://bnc/exp-rob-lab/2022-23#','')
            self.robot_at = robot_at_tem[0]
            print('robot_at_updated')
        else:
            rospy.loginfo('get initial postion error')
        
    def battery_callback(self,msg):
        """
        A callback for acquiring the battery data
 """

        self.battery_capacity = msg.capacity
        if self.battery_capacity <= 500:
            self.battery_low = True
        else:
            self.battery_low = False


    def control_callback(self,msg):
        """
        A callback for receive control command from /control
 """
        if msg.cmd == 'go':
            self.go_work = True
        if msg.cmd == 'stop':
            self.go_work = False

    # this function is used to simulate the movement of robot.
    def move(self,next_v):
        """
        To call /move node to move the robot to target location
 """
        
        try:
            self.Move_client = rospy.ServiceProxy(anm.SVR_MOVE, Move)
            #req_msg = Move()
            #request Armo for the location.
            robot_at_tem = self.client.query.dataprop_b2_ind('pos_X',next_v)
            req_msg_x = float(str(robot_at_tem).split('"')[1])

            robot_at_tem = self.client.query.dataprop_b2_ind('pos_Y',next_v)
            req_msg_y = float(str(robot_at_tem).split('"')[1])
            print('send_move_to client')
            resp1 = self.Move_client(req_msg_x,req_msg_y)
            if resp1.succeeded:
                self.update_pos(next_v)
                self.track += '-->' + next_v
                rospy.loginfo(self.track)
                time.sleep(1)
                print('new_target_reached')
                return resp1.succeeded
            else:
                rospy.loginfo('moving failed')
                return False
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return False


    # After robot moving to a new position, the ontology-location need to be updated.
    def update_pos(self,next_v):
        """
        Update new environment information to ARMOR. Called once the robot move to a new location.
 """
        next_vist = next_v
        next_vist = next_vist.replace('<','')
        next_vist = next_vist.replace('>','')
        next_vist = next_vist.replace('http://bnc/exp-rob-lab/2022-23#','')

        #update current_pos
        curren_pos_visit = 0
        curren_pos_visit = int(datetime.datetime.now().timestamp())
        
        # do not update if it is E, visitedAt of E is set to 9999999999
        #update visit_location time
        if next_vist.find('E') == -1:
            L_visit = self.client.query.dataprop_b2_ind('visitedAt',next_vist)
            if len(L_visit)>0:
                old_string =  ''.join([n for n in L_visit[0] if n.isdigit()])
                if self.client.manipulation.replace_dataprop_b2_ind('visitedAt', next_vist, 'LONG', str(curren_pos_visit),old_string):
                    pass#rospy.loginfo('replace visitedAt success')
                else: 
                    rospy.loginfo('replace visitedAt error')
                time.sleep(0.01)
            else: 
                rospy.loginfo('query visitedAt failed')
                return -1

        #update robot now 
        robot_at_tem = self.client.query.dataprop_b2_ind('now','Robot1')
        if len(robot_at_tem)>0:
            old_string =  ''.join([n for n in robot_at_tem[0] if n.isdigit()])
            if self.client.manipulation.replace_dataprop_b2_ind('now', 'Robot1', 'LONG', str(curren_pos_visit),old_string):
                pass#rospy.loginfo('replace Robot1 success ',)
            else: 
                rospy.loginfo('replace Robot1 now error')

        else: 
            rospy.loginfo('query Robot1 now failed')
            return -1
        
        time.sleep(0.01)
        #'update robot done'
        if self.client.manipulation.replace_objectprop_b2_ind('isIn', 'Robot1', next_vist, self.robot_at):
            #rospy.loginfo('replace isIn success now in %s',next_vist)
            self.robot_at = next_vist
        else: 
            rospy.loginfo('replace isIn error')
            time.sleep(0.01)
        
    
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


        return 0

    def next_go(self):
        """
        Returns the next location that the robot should visit next.  the vising strategy is planted here
 """

        while self.go_work == False:
            try:
                rospy.loginfo('waiting start')
                time.sleep(1)
            except:
                break
        #'get canReach'
        canReach = self.client.query.objectprop_b2_ind('canReach','Robot1')
        urgent =  self.client.query.ind_b2_class('URGENT')
        Urgent = []
        for iter2 in urgent:
            iter2 = iter2.replace('http://bnc/exp-rob-lab/2022-23#','')
            iter2 = iter2.replace('<','')
            iter2 = iter2.replace('>','')
            Urgent.append(iter2)
        # strip can reach
        can_reach = []
        for iter2 in canReach:
            iter2 = iter2.replace('http://bnc/exp-rob-lab/2022-23#','')
            iter2 = iter2.replace('<','')
            iter2 = iter2.replace('>','')
            can_reach.append(iter2)
        #if robot at E and battery_low, stay there.
        if self.battery_low and self.robot_at.find('E') != -1:
            return 'E',0
        
        # E first when battery is low
         # choose where to go when battery low. if the Corridor is not connected to E, it will go to other C
        if self.robot_at.find('C') != -1:
            if self.battery_low:
                for iter3 in can_reach:  # E First
                    if iter3 == 'E':
                        return 'E',1
                for iter4 in can_reach:
                    if iter4.find('C') != -1:
                        return iter4,1

        #'check urgent canReach'
        if len(Urgent)	> 0:
        	for iter in can_reach:
                #inner loop
                 if iter.find('E') != -1:
                     continue
                 for iter1 in Urgent :
                    if iter == iter1:
                        return iter, 1
        
        # choose the one the didn't visit for the biggest amount of time
        short_time = 9999999999
        next_visit = ''
        for iter3 in can_reach:
            if iter3.find('E') != -1:
                continue
            visit_time = self.client.query.dataprop_b2_ind('visitedAt',iter3)
            visit_time_str =  ''.join([n for n in  visit_time[0] if n.isdigit()])
            
            if int(visit_time_str) < short_time:
                short_time = int(visit_time_str)
                next_visit = iter3
           
        
        return next_visit, 0





class wait_map(smach.State):
    """
    A class for smach to represent the initial state of the robot, which is waiting for everything get ready.
    The only action this state can take is to let the robot start working.
    
    Methods -------
    execute()
    this function will wait topological-map get ready, then ask Decision-maker where the robot should go next, then go there.
    """
    def __init__(self,DM):
        self.dm = DM
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['start'])

    def build_map(self):
        i = 0
        while self.dm.go_work == False:
            try:
                i = i+1
                if i%20 == 0:
                    rospy.loginfo('waiting start')
                time.sleep(1)
            except:
                break
        while self.dm.map_built == False:
            try:
                i = i+1
                if i%20 == 0:
                    rospy.loginfo('waiting map_built')
                time.sleep(1)
            except:
                break
        # look around
        
    def execute(self, userdata):
        rospy.wait_for_service('/armor_interface_srv')
        #inform DM to build map and then wait.
        self.build_map()
        [next_go,no_wait] = self.dm.next_go()
        # wait until can go somewhere else
        
        if no_wait == 0:
            time.sleep(DELAY_TIME/2)
        self.dm.move(next_go)
        self.dm.update_pos(next_go)
        return 'start'

#This state is for representing robot stay on the corridor
class wander(smach.State):
    """
    A class for smach to represent the robot is now on the corridor.
    The following actions are 'enter','switch_corridor','go_charge', which represents the robot enter a room, 
    go to another corridor, and go to E and charge itself

    Methods -------
    execute()
    After entering this state, this function will ask Decision-maker where the robot should go next, then go there.
    """

    def __init__(self,DM):
        # initialisation function, it should not wait
        self.dm = DM
        smach.State.__init__(self, 
                             outcomes=['enter','switch_corridor','go_charge']
                            )
    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking
        [next_go,no_wait] = self.dm.next_go()
        if no_wait == 0:
            time.sleep(DELAY_TIME)
        self.dm.move(next_go)
        now_at = next_go
        if self.dm.battery_low and now_at.find('E') != -1:
             
                return 'go_charge'

        if now_at.find('C'):
            return 'switch_corridor'
        if now_at.find('R'):
            return 'enter'
              
        
#This state represent visiting rooms
class checking(smach.State):
    """
    A class for smach to represent the robot is now checking the room. 
    The following action is to leave the room.
    
    Methods -------
    execute()
    After entering this state, this function will ask Decision-maker where the robot should go next, then go there.
    """


    def __init__(self,DM):
        self.dm = DM
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['leave'])
    def execute(self, userdata):
        [next_go,no_wait] = self.dm.next_go()
        if no_wait == 0:
            time.sleep(DELAY_TIME)
        self.dm.move(next_go)

        return 'leave'
# This state represents charging the robot
class charging(smach.State):
    """
    A class for smach to represent the robot is now charging its battery 
    The following action is "restart", which represents the robot go back to work again.

    Methods -------
    execute()
    After entering this state, this function will ask Decision-maker where the robot should go next,
    then go. Also this function will publish message to the manager of the battery to begin charge itself, 
    then wait until fully charged. 
    """
    def __init__(self,DM):
        self.dm = DM
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['restart'])
        
        self.Bat_pub = rospy.Publisher(anm.TOPIC_CHARGE, Charge, queue_size=1, latch=True)
    

    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking
        charge_msg = Charge()
        charge_msg.stamp = rospy.Time.now()
        charge_msg.charge = True
        self.Bat_pub.publish(charge_msg)
        self.dm.go_work = False
        while self.dm.battery_low and (self.dm.go_work == False):
            try:
                rospy.loginfo('charging %d',self.dm.battery_capacity)
                self.Bat_pub.publish(charge_msg)
                time.sleep(2)
            except:
                break
        # Tell battery that state-machine doesn't ask it to charge anymore
        charge_msg.charge = False
        charge_msg.stamp = rospy.Time.now()
        self.Bat_pub.publish(charge_msg)
        self.dm.go_work = True
        [next_go,no_wait] = self.dm.next_go()
        if no_wait == 0:
            time.sleep(DELAY_TIME)
        self.dm.move(next_go)
        return 'restart'

def main():

    """ 
    Main function
    This function will create state-machine, while pass object Decision-Maker as parameter into it.
    Before everything get started, this function will wait ARMOR service and the service to move the robot.

    """

    DM = decision_maker()
    rospy.init_node(anm.NODE_SDM,log_level =rospy.INFO)

    sm = smach.StateMachine(outcomes=['State of robot'])
    sm.userdata.sm_counter = 0

    rospy.wait_for_service(anm.SVR_ARMOR)
    rospy.wait_for_service(anm.SVR_MOVE)

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('wait_map', wait_map(DM), 
                               transitions={'start':'on_corridor'})

        smach.StateMachine.add('on_corridor', wander(DM), 
                               transitions={'enter':'checking', 
                                            'go_charge':'charging', 
                                            'switch_corridor':'on_corridor'})
        smach.StateMachine.add('checking', checking(DM), 
                             transitions={'leave':'on_corridor'})
        smach.StateMachine.add('charging', charging(DM), 
                             transitions={'restart':'on_corridor'})
    # Create a state machine
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    # Execute the state machine
    outcome = sm.execute()
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()



if __name__ == '__main__':
    # Initialise the node, its action server, and wait. 

    server = main()
    #server.run()
    rospy.spin()
