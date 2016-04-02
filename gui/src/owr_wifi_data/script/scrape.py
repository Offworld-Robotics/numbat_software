#!/usr/bin/env python
import rospy
import os
#import requests
#import subprocess 
import telnetlib
import time
import json
import rospy
from owr_messages.msg import status

def run():
    #telnet = subprocess.Popen(["telnet",  "192.168.1.20"], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    #print telnet.communicate("bluetounge\nbluetoung\n/usr/www/signal.cgi");
    connection.open("192.168.1.20")
    connection.read_until("login:")
    connection.write("bluetounge\n")
    connection.read_until("Password:")
    connection.write("bluetoung\n")
    connection.read_until("XM.v5.5.8#")
    connection.write("/usr/www/signal.cgi\n")
    
    data = connection.read_until('}', 2)

    #cleaning header off telnet response
    data = (data.split('{', 1)[-1])
    data = '{' + data
    data = data.strip(" \n\t");
    data_dict = json.loads(data)
    print data_dict['signal']
    message = status()
    message.battery = 5
    message.signal = (data_dict['signal'] + 100)/10.0
    print message.signal
    return message

    
if __name__ == '__main__':
    message = status()
    pub = rospy.Publisher("/status/battery", status, latch=True)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)
    connection = telnetlib.Telnet("192.168.1.20")  #192.168.1.20
    while not rospy.is_shutdown():
        message = run()
        pub.publish(message)
        rate.sleep()
        time.sleep(30)
        
