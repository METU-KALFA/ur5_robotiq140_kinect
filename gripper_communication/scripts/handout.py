#!/usr/bin/env python

from __future__ import print_function

from gripper_communication.srv import Handout, HandoutResponse
from std_msgs.msg import Float64MultiArray, String
import rospy

current_wrench = None
initial_wrench = None

def ftCallback(data):
    global current_wrench
    current_wrench = data.data

def compute_change(initial_wrench, current_wrench):
    change = abs(current_wrench[0] - initial_wrench[0])
    change += abs(current_wrench[1] - initial_wrench[1])
    change += abs(current_wrench[2] - initial_wrench[2])
    return  change
           

def handle_handout(req):
    
    global initial_wrench
    global current_wrench
    
    print("Request is arrived.")

    rospy.Subscriber("ft_data", Float64MultiArray, ftCallback)
    gripper_pub = rospy.Publisher("gripper_control", String)

    while True:
        if initial_wrench is None:
            initial_wrench = current_wrench
            # print(current_wrench)
            continue
        
        change = compute_change(initial_wrench, current_wrench)
        # print(change)
        if change > 20:
            gripper_pub.publish(String("0.1"))
            rospy.sleep(0.2)
            return HandoutResponse(True)

def handout_server():
    rospy.init_node('handout_server')
    s = rospy.Service('handout', Handout, handle_handout)
    print("Ready to handle handout.")
    rospy.spin()

if __name__ == "__main__":
    handout_server()