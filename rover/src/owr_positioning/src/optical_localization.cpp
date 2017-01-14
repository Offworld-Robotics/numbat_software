#include <cmath>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <std_msgs/Header.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include "optical_localization.h"

double OpticalLocalization::angle_between(cv::Mat& a, cv::Mat& b) {
	return atan2(b.at<double>(1), b.at<double>(0)) - atan2(a.at<double>(1), a.at<double>(0));
}

void OpticalLocalization::image_callback(const sensor_msgs::Image::ConstPtr& image) {
	cv::Mat frame = cv_bridge::toCvCopy(image)->image;
	cv::Mat frame_gray;
	cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
	
	if(!prev_gray.empty()) {
		cv::Mat affineTransform = estimateRigidTransform(prev_gray, frame_gray, false); // 2x3 matrix
		if(!affineTransform.empty()) {
			cv::Mat translation_delta = affineTransform.col(2); // get third column (the translation vector)
			double scale_x_delta = norm(affineTransform.col(0)); // get norm of first column
			double scale_y_delta = norm(affineTransform.col(1)); // get norm of second column
			cv::Mat rot1 = (cv::Mat_<double>(2,1) << 0, 1);
			cv::Mat rot2 = affineTransform(cv::Range::all(), cv::Range(0,2)) * rot1;
			
			double rotation_delta = angle_between(rot1, rot2);
			
			translation_current += translation_delta;
			scale_x *= scale_x_delta;
			scale_y *= scale_y_delta;
			rotation += rotation_delta;
			
			double time_scale = 0;
			ros::Time current_time = ros::Time::now();
			if(!prev_time.isZero()) {
				time_scale = (1.0 / ((current_time - prev_time).nsec / 1e9));
			}
			prev_time = current_time;
			
			geometry_msgs::TwistWithCovarianceStamped my_twist;
			my_twist.twist.twist.linear.x = translation_delta.at<double>(0) * linear_velocity_scale * time_scale;
			my_twist.twist.twist.linear.y = translation_delta.at<double>(1) * linear_velocity_scale * time_scale;
			my_twist.twist.twist.linear.z = 0;
			
			my_twist.twist.twist.angular.x = 0;
			my_twist.twist.twist.angular.y = 0;
			my_twist.twist.twist.angular.z = rotation_delta * rotation_velocity_scale * time_scale;
			
			for(unsigned int i = 0;i < 36;++i) {
				my_twist.twist.covariance[i] = 0;
			}
			
			my_twist.header.stamp = ros::Time::now();
			my_twist.header.seq = seq;
			++seq;
			my_twist.header.frame_id = "base_link";
			
			pub.publish(my_twist);
			ROS_INFO_STREAM(my_twist.twist.twist);
		}
	}
	prev_gray = frame_gray;
}

OpticalLocalization::OpticalLocalization(int argc, char *argv[]) :
	pixels_per_metre_traversed(2310),
	linear_velocity_scale(1.0/pixels_per_metre_traversed),
	rotation_velocity_scale(1.0)
	{
	
	translation_current = (cv::Mat_<double>(2,1) << 0, 0);
	scale_x = 1.0;
	scale_y = 1.0;
	rotation = 0.0;
	seq = 0;
	
	ros::init(argc, argv, "optical_localization");
	ros::NodeHandle n("~");
	sub = n.subscribe("/optical_localization_cam/image_raw", 1, &OpticalLocalization::image_callback, this);
	pub = n.advertise<geometry_msgs::TwistWithCovarianceStamped>("/owr/optical_localization_twist", 10, true);
}

void OpticalLocalization::run() {
	while(ros::ok()) {
		ros::spin();
	}
}

int main(int argc, char *argv[]) {
	OpticalLocalization ol(argc, argv);
	ol.run();
	return 0;
}
