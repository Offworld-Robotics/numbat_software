#!/usr/bin/env python
# license removed for brevity
import rospy
import subprocess
from std_msgs.msg import String,Int16

"""in visudo, need to add %ros ALL = NOPASSWD: /home/ros/owr_software/rover/src/owr_usb_reset/usb_reset
after %sudo	ALL=(ALL:ALL) ALL. This allows the usb reset program to run without needing a password for 
sudo. Eneter password manually while running causes a problem 
"""
global devices

def idmap(device):
	if (device[23:27] == "1d6b"):
		return "linux"
	else:
		return device[0:32]

def callback(data):
	print(data.data)
	"""filepath = "/dev/bus/usb" + devices[data.data][0:32]
	rospy.info("Usb reset")
	command = "sudo ./usb_reset " + filepath
	subprocess.call(command,shell=True)
	rospy.loginfo("%s reset", data.data) """

def listener():
	rospy.init_node('usb_reset', anonymous=True)
	rospy.Subscriber("rover/usb/reset", String, callback)

def talker():
	pub = rospy.Publisher('/rover/usb', String, queue_size=1)
	rospy.init_node('usb_reset', anonymous=True)
	rate = rospy.Rate(1) # 1hz

	while not rospy.is_shutdown():
		subprocess.call("cd /dev/bus/usb",shell=True)
		devices = subprocess.check_output(["lsusb"])
		devices = devices.split("\n")
		device_string = idmap(devices[0])
		for i in range(1,len(devices)):
    			device_string = device_string + "#/#/" + idmap(devices[i])
		rospy.loginfo(devices)
		pub.publish(device_string)
		rate.sleep()

if __name__ == '__main__':
	try:
		listener()
		talker()
	except rospy.ROSInterruptException:
		pass





