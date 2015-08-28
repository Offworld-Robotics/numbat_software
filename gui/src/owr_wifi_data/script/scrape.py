#!/usr/bin/env python
import rospy
import os
#import requests
#import subprocess 
import telnetlib
from owr_messages.msg import stream

def run():
    #telnet = subprocess.Popen(["telnet",  "192.168.1.20"], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    #print telnet.communicate("bluetounge\nbluetoung\n/usr/www/signal.cgi");
    connection = telnetlib.Telnet("192.168.1.21")
    connection.read_until("login:")
    connection.write("bluetounge")
    connection.read_until("Password:")
    connection.write("bluetoung")
    connection.read_until("XM.v5.5.8#")
    connection.write("/usr/www/signal.cgi")
    json = connection.read_very_eager()
    print json
    
    pass

    
if __name__ == '__main__':
    run()
