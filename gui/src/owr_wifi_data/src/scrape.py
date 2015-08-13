#!/usr/bin/env python
import rospy
import os
import requests
import subprocess 
from owr_messages.msg import stream

def run():
    telnet = subprocess.Popen(["telnet",  "192.168.1.20"], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    print telnet.communicate("bluetounge\nbluetoung\n/usr/www/signal.cgi");
    pass

    
if __name__ == '__main__':
    run()
