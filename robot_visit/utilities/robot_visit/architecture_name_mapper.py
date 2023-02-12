#!/usr/bin/env python
import rospy


TOPIC_CONTROL = 'state/control'
TOPIC_BATTERY = 'state/battery'
TOPIC_MAP = 'state/map_built'
TOPIC_CHARGE = 'state/charge'
TOPIC_MARKERS = '/aruco_marker_publisher/markers'
NODE_SDM = 'state_decision_maker'
NODE_MOVE = 'move'
NODE_BATTERY = 'battery'
NODE_CONTROL = 'control'
NODE_MAP = 'build_map'
SVR_ARMOR = 'armor_interface_srv'
SVR_MOVE = 'motion/move'
