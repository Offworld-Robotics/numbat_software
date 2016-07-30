#! /usr/bin/env python
import rospy
import os
import telnetlib
import time
import json
import rospy
import dynamic_reconfigure.client


log_file = 0
client = 0
low_bitrate = False  
param_high_bitrate = { 'target_bitrate' : 800000 }
param_low_bitrate = { 'target_bitrate' : 100000 } 

def run():
    global low_bitrate
    connection.open("192.168.1.22")
    connection.read_until("login:")
    connection.write("ubnt\n")
    connection.read_until("Password:")
    connection.write("ubnt\n")
    connection.read_until("XM.v5.5.8#")
    connection.write("/usr/www/signal.cgi\n")

    data = connection.read_until('}', 2)

    data = (data.split('{', 1)[-1])
    data = '{' + data
    data = data.strip(" \n\t");
    data_dict = json.loads(data)

    signal = data_dict['signal']

    print "signal: " + str(signal) + "\n"
    configuration = client.get_configuration()
    print "optimize_for: " + str(configuration['optimize_for'])
    print "target_bitrate: " + str(configuration['target_bitrate'])
    print "configuration: \n"
    print configuration

    if (signal < -60) and not low_bitrate:
        print "low signal strength! Switching to low bit rate\n" 
        low_bitrate = True        
        client.update_configuration(param_low_bitrate)
    elif (signal > -55) and low_bitrate:
        print "normal signal strength--returning to high bit rate\n"
        low_bitrate = False 
        client.update_configuration(param_high_bitrate)    

if __name__ == '__main__':
    print "Running comms_control\n"

    rospy.init_node('dynparam', anonymous=True)  

    client = dynamic_reconfigure.client.Client("/cam0/theora")

    params = { 'optimize_for' : 0 } 
    
    client.update_configuration(params)   
    connection = telnetlib.Telnet("192.168.1.22")
    while not rospy.is_shutdown():
        run()
        time.sleep(5)

