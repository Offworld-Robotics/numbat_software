#!/usr/bin/env python

# Python script to communicate with the
# Agilent U1253A / U1272A / U1273A etc.
# found originally on http://goo.gl/Gycv9H

# For more information on the protocol, check
# http://blog.philippklaus.de/2014/02/agilent-u1273a/
# and http://goo.gl/oIJi96

import sys
import time
import serial
import rospy
import rospkg
from std_msgs.msg import String
from std_msgs.msg import Float32

def init_meter(com_port):
    global meter
    try:
        #print('meter, starting open:')
        meter = serial.Serial(com_port, 9600, timeout=.1)
        time.sleep(1)
        #print('meter,  done open')
        #print(meter)
        #print('meter,  reseting meter:')
        meter.write("RST\n")
        time.sleep(0.25)
        response = meter.read(100)
        #print(response)
        return 0
    except:
        return 1

def read_meter():
    global meter

    #if second != 'yes' :
    #print ('not in second')
    meter.write("FETC?\n")
    #else :
    #print ('yes in second')
    #    meter.write("FETC? @2\n")

    #time.sleep(0.05)
    responsestr = meter.read(17)

    #print ('>' + responsestr + '<', len(responsestr))
    try:
        response = float(responsestr)
    except:
        response = 1337.0
        index = responsestr.find("\n")
        meter.read(index+1)
    return response

def close_meter():
    global meter
    #print('meter, starting close')
    #print(meter)
    meter.close()
    #print('meter, closed')

    return meter


def talker(reading):
    pub.publish(reading)
    rate.sleep()

if __name__=='__main__':
    
    pub = rospy.Publisher("/owr/voltmeter", Float32, queue_size=10)
    rospy.init_node('serial_voltmeter', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    meter = "/dev/voltmeter"
    while(init_meter(meter)) : 
        #print("connecting")
        rate.sleep()
    
    # Constantly read from the voltmeter, and publish as a rostopic
    while 1:
        primaryValue = read_meter()
        #time.sleep(0.1)
        #print( "{:10.4f}".format(primaryValue))
        try:
            talker(primaryValue)
        except rospy.ROSInterruptException:
            pass
    close_meter()
