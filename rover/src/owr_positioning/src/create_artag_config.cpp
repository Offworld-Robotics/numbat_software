/*
   Author : Abhishek Vijayan
   Purpose : Create a launch file containing positions of AR Tags
*/

#include <iostream>
#include <fstream>
using namespace std;

int determine_artag(){
   int id;
   cout << "Enter ARTag Id : ";
   cin >> id;
   return id;
}

int main() {
   ofstream launch_file;
   launch_file.open("artags_location.launch");
   bool add_artag = true;
   char response;
   int id;
   float x, y, z, x_r, y_r, z_r;
   if(launch_file.is_open()){
      launch_file << "<launch>\n";
      while(add_artag){
        id = determine_artag();
        cout << "Enter x, y, z co-ordinates separated by space (or in different lines) : ";
        cin >> x >> y >> z;
        cout << "Enter rotation in x, y, z separated by space (or in different lines) : ";
        cin >> x_r >> y_r >> z_r;
        launch_file << "<node pkg=\"tf\" type=\"static_transform_publisher\" name=\"marker"<< id 
	   <<"\" args=\""
	   << x << " " << y << " " << z << " "
           << x_r << " " << y_r << " " << z_r << " "
           << "world /marker" 
           << id << " 100\">\n";
// <node pkg="tf" type="static_transform_publisher" name="marker0" args="2.0 1.0 0.0 0 0 0 world /marker0 100">
        cout << "Do you want to add details of another AR Tag ? (Y/N) : ";
	cin >> response;
        if(response == 'N'){
	   add_artag = false;
        }
      }
      launch_file << "</launch>\n";	
      launch_file.close(); 
   }
   return 0;
}
