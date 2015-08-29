#!/usr/bin/env python
import rospy
import os
#import requests
#import subprocess 
import telnetlib
import time
import json
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
    #data = data.strip(" \n\t");cd 
    data_dict = json.loads(data)
    status.signal = (-data_dict['signal'])/100.0
    connection.close()
    pass

    
if __name__ == '__main__':
    while True:
	connection = telnetlib.Telnet("192.168.1.20")  #192.168.1.20
    	run()
	time.sleep(3)
