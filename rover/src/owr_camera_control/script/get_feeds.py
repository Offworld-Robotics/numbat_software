#!/usr/bin/env python
import subprocess
import re

import rospy
from owr_messages.msg import activeCameras
from owr_messages.msg import stream


def talker():
    pub = rospy.Publisher('owr/control/availableFeeds', activeCameras, queue_size=10)
    rospy.init_node('camera_feeds', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        cameraList = activeCameras(); # The array of stream structs
        index = 0; # Index for the array of streams, used to contiguously fill cameraList
        
        proc = subprocess.Popen(["ls", "/dev/"], stdout=subprocess.PIPE)
        (out, err) = proc.communicate()
        a = out.split() # Get all directory strings inside /dev/
        for word in a:
            if re.search("video", word): # Check if word is a video device
                nums = re.findall(r'\d+', word)
                
                camNum = int(nums[0]) # Return the number of the video device
                
                # Fill out the information for the stream
                s = stream()
                s.stream = camNum
                s.on = False
                
                cameraList.cameras.append(s)
                index += 1
                
        rospy.loginfo(cameraList)
        pub.publish(cameraList)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
