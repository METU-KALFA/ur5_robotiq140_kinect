#!/usr/bin/env python

import socket
import time
import sys
import rospy
from sensor_msgs.msg import JointState


HOST = "10.0.0.2" # The remote host
PORT = 63352 # The same port as used by the server
def socket_publish(): 
    state_pub = rospy.Publisher("joint_states", JointState, queue_size=1)
    # state_pub = rospy.Publisher("finger_state", JointState, queue_size=1)
    rospy.init_node("gripper_state_publisher",anonymous=True)
    rate = rospy.Rate(20) # 10hz
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.settimeout(10)
        print("Connecting to " + HOST)   
        s.connect((HOST, PORT))
        print("Connected")
        try:
            while (not rospy.is_shutdown()):
                s.send(b"GET POS")
                data = s.recv(1024)
                data = int(data.encode("utf-8"))
                # print(type(data), data) #check type and convert to int
                joint_pos = (float(data)/230) *0.7 # in gazebo position 0.7 means closed gripper, 230 instead of 255
                joint_state = JointState()
                joint_state.header.stamp = rospy.Time.now()
                joint_state.name = ["finger_joint"]
                joint_state.position = [joint_pos]
                state_pub.publish(joint_state)
                rate.sleep()
                # since action requests are also done by the socket connection add them tp wprk with pos request
        except KeyboardInterrupt:
            print ("Exception after connected: " + str(e))

    except Exception as e:
        print ("Exception before connected: " + str(e))
    
    s.close()

if __name__ == "__main__":
    socket_publish()


    