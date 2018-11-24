#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

import usb
import rospy
import os
import string
from std_msgs.msg import String

def is_camera(dev, intf):
        if (dev.bDeviceClass == 14 or intf.bInterfaceClass == 14): return True
        if (dev.bDeviceClass == 16 or intf.bInterfaceClass == 16): return True
        if (dev.bDeviceClass == 6 or intf.bInterfaceClass == 6): return True
        return False

def camera_identifier():
    pub = rospy.Publisher('camera_identication', String, queue_size=10)
    rospy.init_node('camera_identifer', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        str = "\n"
        busses = usb.busses()
        deviceAdded = False
        allDevicesAdded = []
        for bus in busses:
            devices = bus.devices
            devs = usb.core.find(find_all=True)
            for dev in devs:
                deviceAdded = False
            	for cfg in dev:
                    for intf in cfg:
                        for x in allDevicesAdded:
                            uniqueIdentifier = '%s%s' % (dev.idVendor, dev.idProduct)
                            if (x == uniqueIdentifier):
                                deviceAdded = True
                        if (deviceAdded):
                            break
                        if (is_camera(dev, intf)):
                            str += 'Bus %03x, Device %03x, Product: %s| SerialID: %s, Vendor ID: %s, Product ID: %s, Unique Identifier: %d%d \n' % (dev.bus, dev.address,  dev.iProduct, dev.iSerialNumber, dev.idVendor, dev.idProduct, dev.idVendor, dev.idProduct)
                            uniqueIdentifier ='%s%s' % (dev.idVendor, dev.idProduct)
                            allDevicesAdded.append(uniqueIdentifier)
                            break
        rospy.loginfo(str)
        pub.publish(str)
        rate.sleep()


if __name__ == '__main__':
    try:
        camera_identifier()
    except rospy.ROSInterruptException:
        pass
