#include <cmath>
#include <ros/ros.h>
#include <tf2_ros.h>
#include <tf2_geometry_msgs.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Header.h>
#include <opencv2/imgproc.hpp>

class OpticalLocalization {
	private:
		ros::Subscriber sub;
		
		prev_gray;
		auto translation_current[2] = {0.0, 0.0};
		auto scale_x = 1.0;
		auto scale_y = 1.0;
		auto rotation = 0.0;
		//bridge;
		//pub;
		
		constexpr auto pixels_per_metre_traversed = 2310U;
		constexpr auto linear_velocity_scale = 1.0/pixels_per_metre_traversed;
		constexpr auto rotation_velocity_scale = 1.0;
		auto seq = 0U;
		
		double angle_between(double *a, double *b) {
			return arctan2(b[1], b[0]) - arctan2(a[1], a[0]);
		}
		
		void image_callback(const sensor_msgs::Image::ConstPtr& image) {
			auto frame = toCvCopy(image);
			cv2::cvtColor(frame, frame_gray, cv2::COLOR_BGR2GRAY);
			auto vis = frame.copy();
			
			if(!prev_gray.empty()) {
				auto affineTransform = cv2::estimateRigidTransform(prev_gray, frame_gray_false); // 2x3 matrix
				if(!affineTransform.empty()) {
					auto translation_delta = affineTransform; // get third column (the translation vector)
					auto scale_x_delta = norm(affineTransform); // get norm of first column
					auto scale_y_delta = norm(affineTransform); // get norm of second column
					auto rot1 = {0,1};
					auto rot2 = dot(affineTransform[:,[0,1]], {0,1});
					
					auto rotation_delta = angle_between(rot1.A1, rot2.A1);
					
					auto translation_current = translation_current + translation_delta;
					scale_x *= scale_x_delta;
					scale_y *= scale_y_delta;
					rotation += rotation_delta;
					
					time_scale = 0;
					current_time = ros::Time::now();
					if(!prev_time.isZero()) {
						time_scale = (1.0 / ((current_time - prev_time).nsecs / 1e9));
					}
					prev_time = current_time;
					
					TwistWithCovarianceStamped my_twist;
					my_twist.twist.twist.linear.x = translation_delta.item(0,0) * linear_velocity_scale * time_scale;
					my_twist.twist.twist.linear.y = translation_delta.item(1,0) * linear_velocity_scale * time_scale;
					my_twist.twist.twist.linear.z = 0;
					
					my_twist.twist.twist.angular.x = 0;
					my_twist.twist.twist.angular.y = 0;
					my_twist.twist.twist.angular.z = rotation_delta * rotation_velocity_scale * time_scale;
					
					for(auto& i : my_twist.twist.convariance) {
						i = 0;
					}
					
					my_twist.header.stamp = rospy.Time.now();
					my_twist.header.seq = seq;
					++seq;
					my_twist.header.frame_id = "base_link";
					
					pub.publish(my_twist);
					ROS_INFO_STREAM(my_twist.twist.twist);
				}
			}
			prev_gray = frame_gray;
		}
		
	public:
		OpticalLocalization(int argc, char *argv[]) {
			ros::init(argc, argv, "optical_localization");
			ros::NodeHandle n("~");
			sub = n.subscribe("/optical_localization_cam/image_raw", 1, &image_callback, this);
			pub = n.advertise<geometry_msgs::TwistWithCovarianceStamped>("/owr/optical_localization_twist", 10, true);
		}
		
		void run() {
			while(ros::ok()) {
				ros::spin();
			}
		}
	
};

int main(int argc, char *argv[]) {
	OpticalLocalization ol(argc, argv);
	ol.run();
	return 0;
}
