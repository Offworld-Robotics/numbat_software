#! /usr/bin/env python
import rospy
import os
import telnetlib
import time
import json
import rospy
#from owr_messages.msg import status

log_file = 0

def run():
#    connection.open("192.168.1.22")
#    connection.read_until("login:")
#    connection.write("ubnt\n")
#    connection.read_until("Password:")
 #   connection.write("ubnt\n")
#    connection.read_until("XM.v5.5.8#")
#    connection.write("/usr/www/signal.cgi\n")
   log_file.write("signal: \n" ) 
    #print data_dict['signal'], log_file
    #print "rssi: ", log_file
    #print data_dict['rssi'], log_file
    #print "noisef: ", log_file
    #print data_dict['noisef'], log_file

if __name__ == '__main__':
#    rospy.loginfo("Hello there!! running comms_control")
    print "Hello there!! running comms_control\n"

    log_file = open("comms_log.txt", "a") 

    #connection = telnetlib.Telnet("192.168.1.22")
    while not rospy.is_shutdown():
        run()
        time.sleep(5)

    log_file.close()
