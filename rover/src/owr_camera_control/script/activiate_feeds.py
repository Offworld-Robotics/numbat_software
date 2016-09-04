#!/usr/bin/env python
import rospy
import rospkg
import os
import subprocess 
from owr_messages.msg import stream

#called when the message is recived
def callback(data):
    print data
    if data.on:
        rospy.loginfo("turn on /dev/video%d", data.stream)
        rospy.loginfo(rospy.get_param("/rosdistro"))
        if not 'kinetic' in rospy.get_param("/rosdistro"):
            os.environ["GSCAM_CONFIG"] = "v4l2src device=/dev/video"+str(data.stream)+" ! video/x-raw-rgb,framerate=30/1,width=640,height=480 ! ffmpegcolorspace"
            rospy.logerr("not kinetic")
        else:
            os.environ["GSCAM_CONFIG"] = "v4l2src device=/dev/video"+str(data.stream)+" ! videoscale ! video/x-raw,width=640,height=480 ! videoconvert"
        
        print os.environ["GSCAM_CONFIG"]
        #subprocess.Popen([ "rosrun","gscam", "gscam","__name=" + str(data.stream) + "_camera","gscam_publisher:=\/cam" + str(data.stream)])
        subprocess.Popen([ "rosrun","gscam", "gscam","__name:=" + "camera_" + str(data.stream),"/camera/image_raw:=/cam" + str(data.stream)])

    else:
        rospy.loginfo("closing /dev/video%d", data.stream)
        subprocess.call(["rosnode","kill","/camera_" + str(data.stream)]);
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
