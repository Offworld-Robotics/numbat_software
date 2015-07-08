/*
   Buttercup ROS driver
   W.A.
   8/14
*/

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <list>
#include <string>
#include <math.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <iostream>

#include <buttercup_node/buttercup_node.h>
#include <buttercup_node/message.h>

ButtercupNode::ButtercupNode()	
{
	estopPub = nodeHandle.advertise<std_msgs::Bool>("buttercup/estop", 2);
	batteryPub = nodeHandle.advertise<std_msgs::Float32>("buttercup/battery", 2);
	odomPub = nodeHandle.advertise<nav_msgs::Odometry>("buttercup/odometry", 2);
	odomRawPub = nodeHandle.advertise<std_msgs::Float64MultiArray>("buttercup/odom_raw", 2);
	butterPub = nodeHandle.advertise<std_msgs::String>("buttercup/butter", 1, true);

	cmdVelSub = nodeHandle.subscribe("cmd_vel", 1,
			&ButtercupNode::cmdVelCallback, this);
	safetySub = nodeHandle.subscribe("buttercup/safety", 1,
			&ButtercupNode::safetyCallback, this);

	std::string port;
	nodeHandle.param("port", port, std::string("/dev/ttyACM0"));
	ser = openSerial(port);

	double kp, ki, kd;
	nodeHandle.param("P", kp, 0.7);
	nodeHandle.param("I", ki, 0.2);
	nodeHandle.param("D", kd, 0.0);
	P = static_cast<uint16_t>(kp * 32768);
	I = static_cast<uint16_t>(ki * 32768);
	D = static_cast<uint16_t>(kd * 32768);

	nodeHandle.param("width", width, 0.655);
	nodeHandle.param("wheel_circ", wheelCircumference, 8 * 0.0254 * 3.14);

	// odom_raw format: left displacement, right displacement, left velocity, right velocity
	odomRaw.data.resize(4);

	ROS_INFO("Buttercup initialised");

	publishButter("Initialised");

	spin();

	close(ser);
}


void ButtercupNode::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	cmdVelBuf = *msg;
}

void ButtercupNode::safetyCallback(const std_msgs::Int32::ConstPtr& msg)
{
	safety = msg->data;
}

void ButtercupNode::spin()
{
	int rate;
	nodeHandle.param("rate", rate, 20);
	ros::Rate r(rate);

	int safetyPeriod;
	int safetyCounter = 0;
	bool safetyState = 0;
	nodeHandle.param("safety_rate", safetyPeriod, 20);

	while (ros::ok()) {
		switch (safety) {
			case SAFETY_ON:
				safetyState = 1;
				break;
			case SAFETY_FLASHING:
				safetyState = (safetyCounter >= safetyPeriod / 2) ? 1 : 0;
				if (safetyCounter > safetyPeriod) {
					safetyCounter = 0;
				} else {
					safetyCounter++;
				}
				break;
			case SAFETY_OFF:
			default:
				safetyState = 0;
		}
		
		buttercupComm(safetyState);

		ros::spinOnce();
		r.sleep();
	}
}

// Transceive messages with buttercup
void ButtercupNode::buttercupComm(bool safety)
{
	double lVel = (2 * cmdVelBuf.linear.x - width * cmdVelBuf.angular.z) / 2;
	double rVel = (2 * cmdVelBuf.linear.x + width * cmdVelBuf.angular.z) / 2;
	lVel = lVel / wheelCircumference * TICKS_PER_REV * 10;
	rVel = rVel / wheelCircumference * TICKS_PER_REV * 10;
	
	// Send a message
	struct outMessage outBuf;
	outBuf.magic = MESSAGE_MAGIC;
	outBuf.lSpeed = static_cast<int16_t>(lVel);
	outBuf.rSpeed = static_cast<int16_t>(rVel);
	outBuf.P = P;
	outBuf.I = I;
	outBuf.D = D;
	outBuf.out1 = safety;
	
	write(ser, reinterpret_cast<uint8_t *>(&outBuf), sizeof(outBuf));

	// Wait for response. This will take a short time, so sleeping briefly is
	// the easiest thing to do
	usleep(2000);

	struct inMessage inBuf;
	int recd = read(ser, reinterpret_cast<uint8_t *>(&inBuf), sizeof(inBuf));
	if (recd != sizeof(inBuf)) {
		ROS_ERROR_STREAM("Recieved incorrect number of bytes: " << recd);
	} else if (inBuf.magic != MESSAGE_MAGIC) {
		ROS_ERROR("Invalid message header");
	} else {
		estop = inBuf.in2;
		publishBattery(inBuf.vbat, inBuf.vref);
		publishOdom(inBuf.encLDisp, inBuf.encRDisp, inBuf.encLSpeed, inBuf.encRSpeed);
		publishEstop(estop);
	}
}

void ButtercupNode::publishBattery(uint16_t battery, uint16_t reference)
{
	std_msgs::Float32 msg;

	if (reference < 700 || reference > 900) {
		ROS_WARN("Reference voltage out of range");
	} else {
		msg.data = static_cast<double>(battery) / static_cast<double>(reference)
			* 2.49 / 2.7 * 12.7;
		batteryPub.publish(msg);
	}
}

void ButtercupNode::publishEstop(uint16_t estop)
{
	std_msgs::Bool msg;
	msg.data = estop ? true : false;
	estopPub.publish(msg);
}


void ButtercupNode::publishOdom(int32_t lDisp, int32_t rDisp, int16_t lVel, int16_t rVel)
{
	// Covariances from Husky
	const double ODOM_POSE_COVAR_MOTION[] = {1e-3, 0, 0, 0, 0, 0,
											 0, 1e-3, 0, 0, 0, 0,
											 0, 0, 1e6, 0, 0, 0,
											 0, 0, 0, 1e6, 0, 0,
											 0, 0, 0, 0, 1e6, 0,
											 0, 0, 0, 0, 0, 1e6};
	const double ODOM_POSE_COVAR_NOMOVE[] = {1e-9, 0, 0, 0, 0, 0,
											 0, 1e-3, 1e-9, 0, 0, 0,
											 0, 0, 1e6, 0, 0, 0,
											 0, 0, 0, 1e6, 0, 0,
											 0, 0, 0, 0, 1e6, 0,
											 0, 0, 0, 0, 0, 1e-9};

	const double ODOM_TWIST_COVAR_MOTION[] = {1e-3, 0, 0, 0, 0, 0,
											  0, 1e-3, 0, 0, 0, 0,
											  0, 0, 1e6, 0, 0, 0,
											  0, 0, 0, 1e6, 0, 0,
											  0, 0, 0, 0, 1e6, 0,
											  0, 0, 0, 0, 0, 1e6};
	const double ODOM_TWIST_COVAR_NOMOVE[] = {1e-9, 0, 0, 0, 0, 0,
											  0, 1e-3, 1e-9, 0, 0, 0,
											  0, 0, 1e6, 0, 0, 0,
											  0, 0, 0, 1e6, 0, 0,
											  0, 0, 0, 0, 1e6, 0,
											  0, 0, 0, 0, 0, 1e-9};

	static nav_msgs::Odometry odom;

	static double oldDL = 0;
	static double oldDR = 0;

	double absL = lDisp * wheelCircumference / TICKS_PER_REV / 10;
	double absR = rDisp * wheelCircumference / TICKS_PER_REV / 10;

	double l = absL - oldDL;
	double r = absR - oldDR;

	oldDL += l;
	oldDR += r;

	double ds = l + r / 2;
	double da = (r - l) / width;
	
	geometry_msgs::Quaternion q = odom.pose.pose.orientation;	
	double heading = atan2(2 * q.y * q.w, 1 - 2 * q.y * q.y);
	
	heading += da;

	q.w = cos(heading / 2);
	q.y = sin(heading / 2);

	if (q.x != 0 || q.z != 0) {
		ROS_WARN("Orientation is wrong. Probably buttercup odometry code has a bug");
		publishButter("Orientation is wrong. Probably buttercup odometry code has a bug");
	}

	odom.pose.pose.orientation = q;
	odom.pose.pose.position.x += ds * cos(heading);
	odom.pose.pose.position.y += ds * sin(heading);

	double vl = lVel * wheelCircumference / TICKS_PER_REV / 10;
	double vr = rVel * wheelCircumference / TICKS_PER_REV / 10;

	odom.twist.twist.linear.x = (vl + vr) / 2;
	odom.twist.twist.angular.z = (vr - vl) / width;

	// Covariances
	if (vl == 0 && vr == 0) {
		for (int i = 0; i < 36; i++) {
			odom.pose.covariance[i] = ODOM_POSE_COVAR_NOMOVE[i];
			odom.twist.covariance[i] = ODOM_TWIST_COVAR_NOMOVE[i];
		}
	} else {
		for (int i = 0; i < 36; i++) {
			odom.pose.covariance[i] = ODOM_POSE_COVAR_MOTION[i];
			odom.twist.covariance[i] = ODOM_TWIST_COVAR_MOTION[i];
		}
	}

	// Fill raw message
	odomRaw.data[0] = absL;
	odomRaw.data[1] = absR;
	odomRaw.data[2] = vl;
	odomRaw.data[3] = vr;

	// We expect about a average 4ms latency (2ms comms + 2ms context switch)
	odom.header.stamp = ros::Time::now() + ros::Duration(0.004);
	//odomRaw.header.stamp = odom.header.stamp(); // TODO replace this message with the clearpath one that has a timestamp

	odomPub.publish(odom);
	odomRawPub.publish(odomRaw);
}

void ButtercupNode::publishButter(std::string bollocks)
{
	std::string assembly = "\n ";
	for (int i = 0; i < bollocks.length() + 2; i++) {
		assembly += "_";
	}
	assembly += "\n( " + bollocks + " )\n ";
	for (int i = 0; i < bollocks.length() + 2; i++) {
		assembly += "-";
	}
	assembly += "\n    \\   ^__^ \n   "
				"  \\  (oo)\\ ________"
				" \n        (__)\\    "
				"     )\\ /\\ \n      "
				"       ||------w|\n  "
				"           ||      ||";
	std_msgs::String msg;
	msg.data = assembly;
	butterPub.publish(msg);
}

int ButtercupNode::openSerial(std::string port)
{
	int fd = open (port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (fd < 0) {
		ROS_ERROR("Error opening serial port ");
		publishButter("Error opening serial port");
		exit(EXIT_FAILURE);
	}

	struct termios tty;
	struct termios tty_old;
	memset (&tty, 0, sizeof tty);

	/* Error Handling */
	if (tcgetattr(fd, &tty) != 0 )
	{
		ROS_FATAL("Error 1 setting serial port attributes"); //cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << endl;
	}

	/* Save old tty parameters */
	tty_old = tty;

	/* Set Baud Rate */
	cfsetospeed (&tty, (speed_t)BAUD);
	cfsetispeed (&tty, (speed_t)BAUD);

	/* Setting other Port Stuff */
	tty.c_cflag     &=  ~PARENB;        // Make 8n1
	tty.c_cflag     &=  ~CSTOPB;
	tty.c_cflag     &=  ~CSIZE;
	tty.c_cflag     |=  CS8;

	tty.c_cflag     &=  ~CRTSCTS;       // no flow control
	tty.c_cc[VMIN]      =   1;                  // read doesn't block
	tty.c_cc[VTIME]     =   5;                  // 0.5 seconds read timeout
	tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

	/* Make raw */
	cfmakeraw(&tty);

	/* Flush Port, then applies attributes */
	tcflush(fd, TCIFLUSH );
	if (tcsetattr(fd, TCSANOW, &tty) != 0)
	{
		ROS_FATAL("Error 2 setting serial port attributes"); //cout << "Error " << errno << " from tcsetattr" << endl;
	}


	return fd;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "buttercup_node");
	
	ButtercupNode buttercup_node;

	return 0;
}

