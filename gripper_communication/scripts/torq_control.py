#!/usr/bin/env python

import socket
import time
import sys
import rospy
from std_msgs.msg import Float64MultiArray


HOST = "10.0.0.2" # The remote host
PORT = 63351 # The same port as used by the server
def socket_publish(): 
    ft_pub = rospy.Publisher("ft_data", Float64MultiArray, queue_size=1)
    rospy.init_node("ft_sensor",anonymous=True)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        print("Connecting to " + HOST)   
        s.connect((HOST, PORT))
        print("Connected")
        try:
            while (not rospy.is_shutdown()):
                data = s.recv(1024)
                data = data.replace("(","")
                data = data.replace(")","")
                data = data.split(" , ")
                data = [float(datum) for datum in data]
                msg = Float64MultiArray()
                msg.data = data
                ft_pub.publish(msg)
        except KeyboardInterrupt:
            s.close()

    except Exception as e:
        print ("Exception: " + str(e))
    
    s.close()

if __name__ == "__main__":
    socket_publish()