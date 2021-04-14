#!/usr/bin/env python
import json
import rospy
import time 

from std_msgs.msg import String
from python_interface import UR5GripperPythonInteface
from gripper_communication.srv import *

gripper_state = "unknown"
switch_state = "unknown"

def get_gripper_state():
    global ur5_interface
    current_state = ur5_interface.gripper_group.get_current_joint_values()[0]
    if current_state > 0.3:
        return "close"
    else:
        return "open"

def stateCallback(data):
    global switch_state
    switch_state =  data.data 

def wait_control():
    rospy.sleep(0.3)
    while switch_state != "Control":
        rospy.sleep(0.3)

def wait_signal():
    while switch_state != "Signal":
        rospy.sleep(0.1)

ur5_interface = UR5GripperPythonInteface()

state_sub = rospy.Subscriber("switch_state", String, stateCallback, queue_size=1)
state_pub = rospy.Publisher("switch_state", String, queue_size=1)
with open("/home/kovan/ros_workspace/cirak_ws/src/ur5_robotiq140_kinect/ur5_gripper/demo/keyframes.json") as jfile:
    frames = json.load(jfile)

gripper_pub = rospy.Publisher("gripper_control", String, queue_size=1)
# gripper_sub = rospy.Subscriber("gripper_state", String, gripperCallback, queue_size=100)
print("Waiting for handout service")
rospy.wait_for_service("handout")
handout_caller = rospy.ServiceProxy("handout", Handout)

while not (gripper_pub.get_num_connections() > 0):
    rospy.sleep(0.01)

# print(ur5_interface.gripper_group.get_current_joint_values())
ur5_interface.set_velociy_acceleration_factors(0.3, 0.3)

print("START OF DEMO")
print("WAITING GAZE SIGNAL")

idle_config = frames["behavioral"]["idle"]["configuration"]
ur5_interface.execute_joint_state(idle_config)

wait_control()

idle_config = frames["behavioral"]["idle"]["configuration"]
ur5_interface.execute_joint_state(idle_config)
gripper_pub.publish(String("0.6"))

# print("WAITING GAZE SIGNAL")
wait_control()
approach_config = frames["storage"]["conn"]["approach"]
ur5_interface.execute_joint_state(approach_config)

print("APROACHING FIRST OBJECT")
while(get_gripper_state() != "open"):
    gripper_pub.publish(String("0.1"))
    rospy.sleep(0.1)

place_config = frames["storage"]["conn"]["place"]
ur5_interface.execute_joint_state(place_config)

while(get_gripper_state() != "close"):

    gripper_pub.publish(String("0.6"))
    rospy.sleep(0.1)


suspend_config = frames["behavioral"]["suspendx"]["configuration"]
ur5_interface.execute_joint_state(suspend_config)

print("WAITING FOR WORKER GAZE")
state_pub.publish(String("Signal"))
wait_signal()
print(switch_state)
wait_control()

handout_config = frames["behavioral"]["handoutx"]["configuration"]
ur5_interface.execute_joint_state(handout_config)

if(get_gripper_state() != "open"):
    handout_caller()
    rospy.sleep(0.5)

# print(get_gripper_state())
while(get_gripper_state() != "close"):
    gripper_pub.publish(String("0.6"))
    rospy.sleep(0.5)

ur5_interface.execute_joint_state(idle_config)
state_pub.publish(String("Gaze"))
print("TURN FOR MERIC CODE")

###################################################
print("CONN IS HANDOVERED - READY FOR SIDERIGHT")

print("WAITING GAZE SIGNAL")
wait_control()

idle_config = frames["behavioral"]["idle"]["configuration"]
ur5_interface.execute_joint_state(idle_config)
    
print("WAITING GAZE SIGNAL")
wait_control()
approach_config = frames["storage"]["sideRight"]["approach"]
ur5_interface.execute_joint_state(approach_config)

print("APROACHING FIRST OBJECT")
while(get_gripper_state() != "open"):
    gripper_pub.publish(String("0.1"))
    rospy.sleep(0.1)

place_config = frames["storage"]["sideRight"]["place"]
ur5_interface.execute_joint_state(place_config)

while(get_gripper_state() != "close"):
    gripper_pub.publish(String("0.6"))
    rospy.sleep(0.1)


suspend_config = frames["behavioral"]["suspend1"]["configuration"]
ur5_interface.execute_joint_state(suspend_config)

print("WAITING FOR WORKER GAZE")
state_pub.publish(String("Signal"))
wait_signal()
print(switch_state)
wait_control()

handout_config = frames["behavioral"]["handout_new"]["configuration"]
ur5_interface.execute_joint_state(handout_config)

if(get_gripper_state() != "open"):
    handout_caller()
    rospy.sleep(0.5)

while(get_gripper_state() != "close"):
    gripper_pub.publish(String("0.6"))
    rospy.sleep(0.1)

ur5_interface.execute_joint_state(idle_config)
state_pub.publish(String("Gaze"))
print("TURN FOR MERIC CODE")

##################################
print("PART1 IS HANDOVERED - READY FOR PART1")
print("WAITING GAZE SIGNAL")
wait_control()

idle_config = frames["behavioral"]["idle"]["configuration"]
ur5_interface.execute_joint_state(idle_config)
    
print("WAITING GAZE SIGNAL")
wait_control()
approach_config = frames["storage"]["part1"]["approach"]
ur5_interface.execute_joint_state(approach_config)

print("APROACHING FIRST OBJECT")
while(get_gripper_state() != "open"):
    gripper_pub.publish(String("0.1"))
    rospy.sleep(0.1)

place_config = frames["storage"]["part1"]["place"]
ur5_interface.execute_joint_state(place_config)

while(get_gripper_state() != "close"):
    gripper_pub.publish(String("0.6"))
    rospy.sleep(0.1)


suspend_config = frames["behavioral"]["suspend1"]["configuration"]
ur5_interface.execute_joint_state(suspend_config)

print("WAITING FOR WORKER GAZE")
state_pub.publish(String("Signal"))
wait_signal()
print(switch_state)
wait_control()

handout_config = frames["behavioral"]["handout_new"]["configuration"]
ur5_interface.execute_joint_state(handout_config)

if(get_gripper_state() != "open"):
    handout_caller()
    rospy.sleep(0.5)

while(get_gripper_state() != "close"):
    gripper_pub.publish(String("0.6"))
    rospy.sleep(0.1)

ur5_interface.execute_joint_state(idle_config)
state_pub.publish(String("Gaze"))
print("TURN FOR MERIC CODE")

# wait_control()

# print("TESEKKURLER ROBOT ADAM")
# rospy.sleep(1)
# print("DEMOYU YAPMAMIZA IZIN VERDIGIN ICIN")
# rospy.sleep(1)
# print("VE COK TATLI BIR ROBOT OLDUGUN ICIN")
# rospy.sleep(1)