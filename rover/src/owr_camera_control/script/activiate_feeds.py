#!/usr/bin/env python
import rospy
import os
import subprocess 
from owr_messages.msg import stream

camera_list = ("cam_left", "cam_right", "cam_arm", "cam_add_name");



#called when the message is recived
def callback(data):
    print data
    
    
    if data.on:
        rospy.loginfo("turn on %s", camera_list[data.stream]) 
        
        os.environ["GSCAM_CONFIG"] = "v4l2src device=dev/" + camera_list[data.stream] + " ! video/x-raw-rgb,framerate=30/1,width=320,height=240 ! ffmpegcolorspace"
        #subprocess.Popen([ "rosrun","gscam", "gscam","__name=" + str(data.stream) + "_camera","gscam_publisher:=\/cam" + str(data.stream)])
        subprocess.Popen([ "rosrun","gscam", "gscam","__name:=" + camera_list[data.stream],"/camera/image_raw:=/cam" + str(data.stream)])

    else:
        rospy.loginfo("closing %s", camera_list[data.stream])
        subprocess.call(["rosnode","kill",camera_list[data.stream]]);
        #subprocess.call(["rosnode","kill","gscam_publisher/cam" + str(data.stream)]);

        return
        
def activateFeeds():
    rospy.init_node('activateFeeds')
    rospy.Subscriber("owr/control/activateFeeds", stream, callback)
    print "running"
    
    #loops while ros is running
    rospy.spin()
    
    
if __name__ == '__main__':
    rospy.loginfo("running activate_feeds")
    activateFeeds()
