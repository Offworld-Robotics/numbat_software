#!/usr/bin/env python
import subprocess
import re

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
	feedList = [False, False, False, False]
	proc = subprocess.Popen(["ls", "/dev/"], stdout=subprocess.PIPE)
	(out, err) = proc.communicate()
	a = out.split()
	for word in a:
		if re.search("video", word):
			if re.search("0", word):
				feedList[0] = True
			elif re.search("1", word):
				feedList[1] = True
			elif re.search("2", word):
				feedList[2] = True
			elif re.search("3", word):
				feedList[3] = True
        rospy.loginfo(feedList)
        pub.publish(feedList)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
