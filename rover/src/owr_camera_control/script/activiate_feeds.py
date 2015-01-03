#!/usr/bin/env python
import rospy
from owr_camera_control.msg import stream

#called when the message is recived
def callback(data):
    print data

def activateFeeds():
    rospy.init_node('activateFeeds')
    rospy.Subscriber("control/activateFeeds", stream, callback)
    
    #loops while ros is runnign
    rospy.spin()
    
    
if __name__ == '__main__':
    activateFeeds()
