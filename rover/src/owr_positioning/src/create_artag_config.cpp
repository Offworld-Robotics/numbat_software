/*
   Author : Abhishek Vijayan
   Purpose : Create a launch file containing positions of AR Tags
*/

#include "artags.h"
#include <string>
#include <cstdlib>
#include <fstream>

//#define DIR_LOCATION = "/home/ros/owr_software/rover/src/owr_positioning/launch/";
//#define FILE_NAME  = "artags_location.launch";

std::string dir_location = "/home/ros/owr_software/rover/src/owr_positioning/launch/";
std::string file_name = "artags_location.launch";
std::ofstream launch_file;
bool add_artag = true; 

int main(int argc, char ** argv) {
//std::string dir_location = "/home/ros/owr_software/rover/src/owr_positioning/launch/";
//std::string file_name = "artags_location.launch";

    ROS_INFO("Launch file creator started");
    ros::init(argc, argv, "arTag_localization"); 

    ::launch_file.open((::dir_location + ::file_name).c_str());
    if(not ::launch_file){
       std::cout << "Could not create " << ::dir_location << ::file_name <<"\n" ;
       return 1;
    }
    else{
    	::launch_file << "<launch>\n";
    }
    artag_localization arl;
    arl.run();
    if(::launch_file.is_open()){
    	::launch_file << "</launch>\n";
    	::launch_file.close(); 
    	std::cout << "Successfully created " << ::dir_location << ::file_name << "\n";
    }
    return 0;
}

void artag_localization::run() {
    while(ros::ok() && ::add_artag) {
      ros::spinOnce();
    }
}

artag_localization::artag_localization() : tfBuffer(), tfListener(tfBuffer)  {
    //subscribe to the topic that provides the pose of observed ar tags
    sub = nh.subscribe("/ar_pose_marker", 1, &artag_localization::callback, this);
}

void artag_localization::callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
    //find the amount of tags found

    int size = msg->markers.size();
    int id;
    float x, y, z, x_r, y_r, z_r;
    char response;

    //for now we will only look at the first tag
    if (size != 0){
        //get the id of the first tag
        id = msg->markers[0].id;

   	if(::launch_file.is_open() && id != 255){
		std::cout << "Detected ARTag : " << id << "\n";
        	std::cout << "Enter x, y, z co-ordinates separated by space (or in different lines) : ";
        	std::cin >> x >> y >> z;
        	std::cout << "Enter rotation in x, y, z separated by space (or in different lines) : ";
        	std::cin >> x_r >> y_r >> z_r;
        	::launch_file << "<node pkg=\"tf\" type=\"static_transform_publisher\" name=\"marker"<< id 
           		<<"\" args=\""
           		<< x << " " << y << " " << z << " "
           		<< x_r << " " << y_r << " " << z_r << " "
           		<< "world /marker" 
           		<< id << " 100\">\n";
// <node pkg="tf" type="static_transform_publisher" name="marker0" args="2.0 1.0 0.0 0 0 0 world /marker0 100">
      		std::cout << "Do you want to add details of another AR Tag ? (Y/N) : ";
      		std::cin >> response;
      		if(response == 'N'){
              		::add_artag = false;
      		}
      	}

    }

}
