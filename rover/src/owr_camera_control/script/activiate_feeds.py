#!/usr/bin/env python
import rospy
import os
import subprocess 
import time
from owr_messages.msg import stream
from sensor_msgs.msg import CameraInfo
from sensor_msgs.srv import SetCameraInfo
import rospy

#called when the message is recived
def callback(data):
    print data
    if data.on:
        rospy.loginfo("turn on /dev/video%d", data.stream)
        os.environ["GSCAM_CONFIG"] = "v4l2src device=/dev/video"+str(data.stream)+" ! video/x-raw-rgb,framerate=30/1,width=320,height=240 ! ffmpegcolorspace"
        #subprocess.Popen([ "rosrun","gscam", "gscam","__name=" + str(data.stream) + "_camera","gscam_publisher:=\/cam" + str(data.stream)])
        subprocess.Popen([ "rosrun",
                          "gscam",
                          "gscam",
                          "__name:=" + "camera%d"%(data.stream),
                          "/camera/image_raw:=/cam" + str(data.stream),
                          "_frame_id:=/camera%d"%(data.stream),
                          "_camera_info_url:=package://owr_camera_control/calibration/cam%d.yaml"%(data.stream)
        ])
        #time.sleep(1)
        #setCamInfo = rospy.ServiceProxy('/set_camera_info', SetCameraInfo)
        #cam = CameraInfo()
        #cam.header.frame_id = "camera%d"%(data.stream)
        ##NOTE: Make sure to change these if you change tthe stream size
        #cam.width = 320
        #cam.height = 240
        #resp = setCamInfo(cam)
        #rospy.loginfo("resp:" + str(resp))
        
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
