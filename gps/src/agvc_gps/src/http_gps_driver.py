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
#roslib.load_manifest('nmea_gps_driver')
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

import urllib2
import re

if __name__ == "__main__":
    #init publisher
    rospy.init_node('http_gps_driver')
    gpspub = rospy.Publisher('fix', NavSatFix)
    gpsVelPub = rospy.Publisher('vel',TwistStamped)
    gpstimePub = rospy.Publisher('time_reference', TimeReference)
    #Init GPS port
    frame_id = rospy.get_param('~frame_id','gps')
    if frame_id[0] != "/":
        frame_id = addTFPrefix(frame_id)

    time_ref_source = rospy.get_param('~time_ref_source', frame_id)
    navData = NavSatFix()
    gpsVel = TwistStamped()
    gpstime = TimeReference()
    gpstime.source = time_ref_source
    navData.header.frame_id = frame_id
    gpsVel.header.frame_id = frame_id
    GPSLock = False
    try:
        #Read in GPS
        while not rospy.is_shutdown():
            #read GPS line
            req = urllib2.Request('http://192.168.3.16/prog/show?position')
            response = urllib2.urlopen(req)
            data = response.read()

            rospy.sleep(0.5)

            timeNow = rospy.get_rostime()
            fields = data.split('\n')
            print len(fields)
            for i in fields:
                #i = i.strip(',')
                print i

                r = re.match("Latitude\s+([-\d\.]*) deg", i)
                if r is not None:
                    latitude = float(r.group(1))
                    print latitude

                r = re.match("Longitude\s+([-\d\.]*) deg", i)
                if r is not None:
                    longitude = float(r.group(1))
                    print longitude

                r = re.match("Altitude\s+([-\d\.]*) meters", i)
                if r is not None:
                    altitude = float(r.group(1))
                    print altitude

                r = re.match("HDOP\s+([-\d\.]*)", i)
                if r is not None:
                    hdop = float(r.group(1))
                    print hdop
                   
                   
                r = re.match("WeekSeconds\s+([\d\.]*)", i)
                if r is not None:
                    weekseconds = float(r.group(1))
                    print weekseconds


            try:
                        navData.status.service = NavSatStatus.SERVICE_GPS

                        navData.header.stamp = timeNow

                        navData.latitude = latitude

                        if latitude == 0 and longitude == 0:
                            navData.status.status = NavSatStatus.STATUS_NO_FIX
                        else:
                            navData.status.status = NavSatStatus.STATUS_GBAS_FIX

                        navData.position_covariance[0] = hdop**2
                        navData.position_covariance[4] = hdop**2
                        navData.position_covariance[8] = (2*hdop)**2 #FIX ME
                        navData.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

                        #Altitude is above ellipsoid, so adjust for mean-sea-level
                        navData.altitude = altitude

                        gpstime.header.stamp = timeNow
                        timeref = "{0:02d}{1:02d}{2:02d}".format(int((weekseconds % 86400) / 3600), int((weekseconds % 3600)/60), int(weekseconds % 60))
                        print timeref
                        gpstime.time_ref = convertNMEATimeToROS(timeref)

                        gpspub.publish(navData)
                        gpstimePub.publish(gpstime)
            except ValueError as e:
                rospy.logwarn("Value error, likely due to missing fields in the NMEA messages. Error was: %s" % e)

    except rospy.ROSInterruptException:
        0
