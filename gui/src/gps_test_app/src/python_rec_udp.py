#!/usr/bin/env python
#Reads gps data from steph's gps app
#Based on Steph's code
import rospy
from std_msgs.msg import String
from std_msgs.msg import Header
from sensor_msgs.msg import NavSatFix

import socket
import re



    
    
def talker():
    UDP_IP = "192.168.0.102"
    UDP_PORT = 9002

    sock = socket.socket(socket.AF_INET, # Internet
                         socket.SOCK_DGRAM) # UDP
    sock.bind((UDP_IP, UDP_PORT))
    pub = rospy.Publisher('gps/fix', NavSatFix, queue_size=10)
    rospy.init_node('test_relay', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
       data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
       print "received message:", data
       data = re.sub("[A-Za-z]","",str(data))
       result = str(data).split(":")
       print result
       rospy.loginfo(result)
       
       
       msg = NavSatFix()
       msg.header = Header()
       msg.latitude = float(result[1])
       msg.longitude = float(result[2])
       #rospy.loginfo(msg)
       pub.publish(msg)
       #rate.sleep()

if __name__ == '__main__':
    try:
       talker()
    except rospy.ROSInterruptException:
       pass
