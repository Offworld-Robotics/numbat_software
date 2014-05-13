#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Steven Martin, Eric Perko
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
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
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

import roslib
roslib.load_manifest('nmea_gps_driver')
import rospy
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from sensor_msgs.msg import TimeReference
from geometry_msgs.msg import TwistStamped

import serial, string, math, time, calendar

#nmea_utc should be a string of form hhmmss
def convertNMEATimeToROS(nmea_utc):
    #Get current time in UTC for date information
    utc_struct = time.gmtime() #immutable, so cannot modify this one
    utc_list = list(utc_struct)
    hours = int(nmea_utc[0:2])
    minutes = int(nmea_utc[2:4])
    seconds = int(nmea_utc[4:6])
    utc_list[3] = hours
    utc_list[4] = minutes
    utc_list[5] = seconds
    unix_time = calendar.timegm(tuple(utc_list))
    return rospy.Time.from_sec(unix_time)

#Add the tf_prefix to the given frame id
def addTFPrefix(frame_id):
    prefix = ""
    prefix_param = rospy.search_param("tf_prefix")
    if prefix_param:
        prefix = rospy.get_param(prefix_param)
        if prefix[0] != "/":
            prefix = "/%s" % prefix

    return "%s/%s" % (prefix, frame_id)

#Check the NMEA sentence checksum. Return True if passes and False if failed
def check_checksum(nmea_sentence):
    split_sentence = nmea_sentence.split('*')
    transmitted_checksum = split_sentence[1].strip()
    
    #Remove the $ at the front
    data_to_checksum = split_sentence[0][1:]
    checksum = 0
    for c in data_to_checksum:
        checksum ^= ord(c)

    return ("%02X" % checksum)  == transmitted_checksum.upper()

if __name__ == "__main__":
    #init publisher
    rospy.init_node('nmea_gps_driver')
    gpspub = rospy.Publisher('fix', NavSatFix)
    gpsVelPub = rospy.Publisher('vel',TwistStamped)
    gpstimePub = rospy.Publisher('time_reference', TimeReference)
    #Init GPS port
    GPSport = rospy.get_param('~port','/dev/ttyUSB0')
    GPSrate = rospy.get_param('~baud',4800)
    frame_id = rospy.get_param('~frame_id','gps')
    if frame_id[0] != "/":
        frame_id = addTFPrefix(frame_id)

    time_ref_source = rospy.get_param('~time_ref_source', frame_id)
    useRMC = rospy.get_param('~useRMC', False)
    #useRMC == True -> generate info from RMC+GSA
    #useRMC == False -> generate info from GGA
    navData = NavSatFix()
    gpsVel = TwistStamped()
    gpstime = TimeReference()
    gpstime.source = time_ref_source
    navData.header.frame_id = frame_id
    gpsVel.header.frame_id = frame_id
    GPSLock = False
    try:
        GPS = serial.Serial(port=GPSport, baudrate=GPSrate, timeout=2)
        #Read in GPS
        while not rospy.is_shutdown():
            #read GPS line
            data = GPS.readline()

            if not check_checksum(data):
                rospy.logerr("Received a sentence with an invalid checksum. Sentence was: %s" % data)
                continue

            timeNow = rospy.get_rostime()
            fields = data.split(',')
            for i in fields:
                i = i.strip(',')
            try:
                if useRMC:
                    #Check for satellite lock
                    if 'GSA' in fields[0]:
                        lockState = int(fields[2])
                        #print 'lockState=',lockState
                        if lockState == 3:
                            GPSLock = True
                        else:
                            GPSLock = False
                    #if not satellite lock message parse it separately
                    else:
                        if GPSLock == True:
                            if 'RMC' in fields[0]:
                                #print fields
                                gpsVel.header.stamp = timeNow
                                gpsVel.twist.linear.x = float(fields[7])*0.514444444444*math.sin(float(fields[8]))
                                gpsVel.twist.linear.y = float(fields[7])*0.514444444444*math.cos(float(fields[8]))
                                gpsVelPub.publish(gpsVel)

                                navData.status.status = NavSatStatus.STATUS_FIX
                                navData.header.stamp = gpsVel.header.stamp
                                navData.status.service = NavSatStatus.SERVICE_GPS

                                gpstime.header.stamp = gpsVel.header.stamp
                                gpstime.time_ref = convertNMEATimeToROS(fields[1])

                                longitude = float(fields[5][0:3]) + float(fields[5][3:])/60
                                if fields[6] == 'W':
                                    longitude = -longitude

                                latitude = float(fields[3][0:2]) + float(fields[3][2:])/60
                                if fields[4] == 'S':
                                    latitude = -latitude

                                #publish data
                                navData.latitude = latitude
                                navData.longitude = longitude
                                navData.altitude = float('NaN')
                                navData.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
                                gpspub.publish(navData)
                                gpstimePub.publish(gpstime)
                        else:
                            pass
                            #print data
                else:
                    #Use GGA
                    #No /vel output from just GGA
                    if 'GGA' in fields[0]:
                        gps_quality = int(fields[6])
                        if gps_quality == 0:
                            navData.status.status = NavSatStatus.STATUS_NO_FIX
                        elif gps_quality == 1:
                            navData.status.status = NavSatStatus.STATUS_FIX
                        elif gps_quality == 2:
                            navData.status.status = NavSatStatus.STATUS_SBAS_FIX
                        elif gps_quality in (4,5):
                            #Maybe 2 should also sometimes be GBAS... but pretty
                            #sure RTK has to have a base station
                            navData.status.status = NavSatStatus.STATUS_GBAS_FIX
                        else:
                            navData.status.status = NavSatStatus.STATUS_NO_FIX
                        navData.status.service = NavSatStatus.SERVICE_GPS

                        navData.header.stamp = timeNow

                        latitude = float(fields[2][0:2]) + float(fields[2][2:])/60
                        if fields[3] == 'S':
                            latitude = -latitude
                        navData.latitude = latitude

                        longitude = float(fields[4][0:3]) + float(fields[4][3:])/60
                        if fields[5] == 'W':
                            longitude = -longitude
                        navData.longitude = longitude

                        hdop = float(fields[8])
                        navData.position_covariance[0] = hdop**2
                        navData.position_covariance[4] = hdop**2
                        navData.position_covariance[8] = (2*hdop)**2 #FIX ME
                        navData.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

                        #Altitude is above ellipsoid, so adjust for mean-sea-level
                        altitude = float(fields[9]) + float(fields[11])
                        navData.altitude = altitude


                        gpstime.header.stamp = timeNow
                        gpstime.time_ref = convertNMEATimeToROS(fields[1])

                        gpspub.publish(navData)
                        gpstimePub.publish(gpstime)
            except ValueError as e:
                rospy.logwarn("Value error, likely due to missing fields in the NMEA messages. Error was: %s" % e)

    except rospy.ROSInterruptException:
        GPS.close() #Close GPS serial port
