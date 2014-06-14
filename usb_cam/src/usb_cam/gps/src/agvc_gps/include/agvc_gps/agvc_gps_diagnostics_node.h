
#ifndef AGVC_GPS_DIAGNOSTICS_NODE_H_
#define AGVC_GPS_DIAGNOSTICS_NODE_H_

#include <ros/ros.h>

#include <sensor_msgs/NavSatFix.h>

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

class AGVCGPSDiagnosticsNode
{

public:

  AGVCGPSDiagnosticsNode();

  void spin();

private:

  void navSatFixCallback(const sensor_msgs::NavSatFix::ConstPtr & nav_sat_fix);

  void diagnoseGPS(diagnostic_updater::DiagnosticStatusWrapper & status);

  ros::NodeHandle node_handle_;

  ros::Subscriber gps_sub_;

  diagnostic_updater::Updater diag_updater_;

  diagnostic_updater::HeaderlessTopicDiagnostic freq_monitor_;

  sensor_msgs::NavSatStatus prev_nav_sat_status_;

  double required_freq_;

};

#endif

