#!/usr/bin/env python
# license removed for brevity
import rospy
import subprocess
from std_msgs.msg import String,Int16
from owr_messages.msg import devices

"""in visudo, need to add %ros ALL = NOPASSWD: /home/ros/owr_software/rover/src/owr_usb_reset/usb_reset
after %sudo	ALL=(ALL:ALL) ALL. This allows the usb reset program to run without needing a password for 
sudo. Eneter password manually while running causes a problem 
"""
#global devs

def idmap(device):
	if (device[23:27] == "1d6b"):
		return "linux"
	else:
		return device[0:32]

def callback(data):
	subprocess.call("cd /dev/bus/usb",shell=True)
	devs = subprocess.check_output(["lsusb"])
	devs = devs.split("\n")
	if(data.data < len(devs) - 1): 
		devs[data.data].replace(' ', '')
		print(devs[data.data])
		filepath = "/dev/bus/usb/" + devs[data.data][4:7]
		filepath = filepath + "/" + devs[data.data][15:18]
		print(filepath)
		command = "sudo ./usb_reset " + filepath
		subprocess.call(command,shell=True)
		rospy.loginfo("%s reset", data.data) 
	else:
		print("Out of range")

def listener():
	rospy.init_node('usb_reset', anonymous=True)
	rospy.Subscriber("rover/usb/reset", Int16, callback)

def talker():
	pub = rospy.Publisher('/rover/usb', devices, queue_size=1)
	rospy.init_node('usb_reset', anonymous=True)
	rate = rospy.Rate(1) # 1hz

	while not rospy.is_shutdown():
		subprocess.call("cd /dev/bus/usb",shell=True)
		devs = subprocess.check_output(["lsusb"])
		devs = devs.split("\n")
		#device_string = idmap(devices[0])
		for i in range(0,len(devs)):
    			devs[i] = idmap(devs[i])
		#rospy.loginfo(device_string)
		pub.publish(devs)
		rate.sleep()

if __name__ == '__main__':
	try:
		listener()
		talker()
	except rospy.ROSInterruptException:
		pass





