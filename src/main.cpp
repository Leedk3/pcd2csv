// inlcude ROS library
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
// include C library
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
// include pcl library
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <point_types.h>


int main (int argc, char** argv) 
{
	ros::init(argc, argv, "pcd2csv");
    ros::NodeHandle nh("~");   

   // Declare important variables
	std::string pcdFile, FullpcdFile;
	std::string csvFile, FullcsvFile;
	std::string timestamp;
	pcl::PointCloud<pcl::PointXYZI> cloud;
	std::ofstream outFile;

	nh.param<std::string>("/pcd2csv_node/pcd_file_name", pcdFile, std::string("trajectory.pcd"));
	nh.param<std::string>("/pcd2csv_node/csv_file_name", csvFile, std::string("trajectory.csv"));

	std::string package_path  = ros::package::getPath("pcd2csv");

	FullpcdFile = (package_path + "/pcd/" + pcdFile).c_str();
	FullcsvFile = (package_path + "/pcd/" + csvFile).c_str();

	// std::cout << "" << pcdFile << "..." << std::endl;
	std::cout << "Loading pcd File  ... \n"<< FullpcdFile << std::endl;

// // Parse input
// 	if (argc==1) {
// 		cerr << "Too few input arguments!" << endl;
// 		cout << "Usage: pcd2csv  pcdInput.pcd" << endl;
// 		cout << "Exiting." << endl;
// 		return(-1);
// 	}
// 	else {
// // Extract timestamp and store I/O filenames		
// 		pcdFile = argv[1];
// 		size_t type_idx = pcdFile.find_last_of(".");
// 		size_t dir_idx = pcdFile.find_last_of("/");
// 		string temp_str = pcdFile.substr(0, type_idx);
// 		timestamp = temp_str.substr(dir_idx+1, temp_str.length());
// 		csvFile = temp_str + ".csv";
// 	}

// // Load in point cloud data
// 	cout << "Loading " << pcdFile << "..." << endl;
	int fileLoad = pcl::io::loadPCDFile(FullpcdFile, cloud);
	if (fileLoad == -1) {
		std::cerr << pcdFile << "not found. Exiting." << std::endl;
		return(-1);
	}
	else {
		std::cout << "File loaded. Converting..." << std::endl;
	}

// // Store in file
	outFile.open(FullcsvFile);
	outFile << "x," << "y," << "z,";
	outFile << "yaw,"  << "velocity," << "change_flag,";
	outFile << "steering_flag," << "accel_flag,";
	outFile << "stop_flag," << "event_flag" << std::endl;
	float x_prev, y_prev;
	x_prev = 0;
	y_prev = 0;
	for (size_t i = 0; i < cloud.width; ++i) {
		float x = cloud.points[i].x;
		float y = cloud.points[i].y;
		float z = cloud.points[i].z;
		float yaw = atan2(y - y_prev, x- x_prev);
		float velocity = 5;
		float intensity = cloud.points[i].intensity; //index
		// int ring = cloud.points[i].ring;
		float az = round(18000*(atan2(x,y)+M_PI)/M_PI);
		float dist = sqrt(x*x+y*y+z*z);
	    outFile << x << "," << y << "," << z << ",";
	    outFile << yaw <<  "," << velocity << "," << intensity << ",";
	    outFile << 0 << "," << 0 << "," << 0 << "," << 0 << std::endl;
		x_prev = x;
		y_prev = y;
	}
	outFile.close();	
  std::cout << "Conversion done! Outputing: " << FullcsvFile << std::endl;
  return (0);
}