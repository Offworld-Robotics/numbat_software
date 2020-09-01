#!/usr/bin/env python

## Uniquely Identifies the camera devices
## If there are issues regarding accessing the full serial number for the cameras, these links may help:
## http://web.archive.org/web/20120629230817/http://ruggedcircuits.com/html/linux.html
## https://askubuntu.com/questions/112568/how-do-i-allow-a-non-default-user-to-use-serial-device-ttyusb0/112572#112572
import usb
import rospy
import hashlib
import os
import string
from std_msgs.msg import String


#checks if the device has the capacity to be a camera
#for the cameras used on the rover, the first if statement should be sufficient
def isCamera(dev, intf):
        if (dev.bDeviceClass == 14 or intf.bInterfaceClass == 14): return True
        #if (dev.bDeviceClass == 16 or intf.bInterfaceClass == 16): return True
        #if (dev.bDeviceClass == 6 or intf.bInterfaceClass == 6): return True
        return False

def getUniqueIdentifier(dev):
    if (len(dev.langids) != 0):
        serial_number = usb.util.get_string(dev, dev.iSerialNumber)
    else:
        serial_number = 0

def cameraIdentifier():
    pub = rospy.Publisher('camera_identication', String, queue_size=10)
    rospy.init_node('camera_identifer', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        info = "\n"
        device_added = False
        all_devices_added = []
        serial_numbers = []
        devices = usb.core.find(find_all=True) #finds all devices connected to the system
        for dev in devices:
            device_added = False
            for cfg in dev:
                for intf in cfg:
                    if (isCamera(dev, intf)):
                        serial_number = getUniqueIdentifier(dev)
                        info += 'Bus %03x, Device %03x, Product: %s| Vendor ID: %s, Product ID: %s, Unqiue Identifer(SN): %s)\n' % (dev.bus, dev.address,  dev.iProduct, dev.idVendor, dev.idProduct, serial_number)
                        allDevicesAdded.append(serial_number)
                        device_added = True
                        break
                if (device_added == True):
                    break
        rospy.loginfo(info)
        pub.publish(info)
        rate.sleep()


if __name__ == '__main__':
    try:
        cameraIdentifier()
    except rospy.ROSInterruptException:
        pass
