/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <time.h>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

#include "MapPoint.h"
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <Converter.h>

//! parameters
bool read_from_topic = false, read_from_camera = false;
std::string image_topic = "/camera/image_raw";
int all_pts_pub_gap = 0;

vector<string> vstrImageFilenames;
vector<double> vTimestamps;
cv::VideoCapture cap_obj;

bool pub_all_pts = false;
int pub_count = 0;

//modified
class ImageGrabber{
public:
	ImageGrabber(ORB_SLAM2::System &_SLAM):
		SLAM(_SLAM), frame_id(0){}

	void GrabImage(const sensor_msgs::ImageConstPtr& msg);

	ORB_SLAM2::System &SLAM;
	int frame_id;
};
bool parseParams(int argc, char **argv);

using namespace std;

int main(int argc, char **argv){
	ros::init(argc, argv, "Monopub");
	ros::start();
	if (!parseParams(argc, argv)) {
		return EXIT_FAILURE;
	}
	int n_images = vstrImageFilenames.size();

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true, true);
	ros::NodeHandle nodeHandler;
	//ros::Publisher pub_cloud 			= nodeHandler.advertise<sensor_msgs::PointCloud2>("cloud_in", 1000)

	if (read_from_topic) {
		ImageGrabber igb(SLAM);
		ros::Subscriber sub = nodeHandler.subscribe(image_topic, 150, &ImageGrabber::GrabImage, &igb);
		ros::spin();
	}
	else{
		cout << "no ROS topic" << endl;
	}
	//ros::spin();

	mkdir("results", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	SLAM.getMap()->Save("results//map_pts_out.obj");
	SLAM.getMap()->SaveWithTimestamps("results//map_pts_and_keyframes.txt");
	// Save camera trajectory
	SLAM.SaveKeyFrameTrajectoryTUM("results//key_frame_trajectory.txt");


	// Stop all threads
	SLAM.Shutdown();

	ros::shutdown();
	return 0;
}

inline bool isInteger(const std::string & s){
	if (s.empty() || ((!isdigit(s[0])) && (s[0] != '-') && (s[0] != '+'))) return false;

	char * p;
	strtol(s.c_str(), &p, 10);

	return (*p == 0);
}


void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg){
	// Copy the ros image message to cv::Mat.
	cv_bridge::CvImageConstPtr cv_ptr;
	try{
		cv_ptr = cv_bridge::toCvShare(msg);
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	SLAM.TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());

	// cv_ptr->header : PoseWithCovarianceStamped needed
	
	//publish(SLAM, pub_pts_and_pose, pub_all_kf_and_pts, pub_pose, frame_id);
	++frame_id;
}

bool parseParams(int argc, char **argv) {
	if (argc < 4){
		cerr << endl << "Usage: rosrun ORB_SLAM2 Monopub path_to_vocabulary path_to_settings path_to_sequence/camera_id/-1 <image_topic>" << endl;
		return 1;
	}

	// check whether load images from disk or from live cam
	if (isInteger(std::string(argv[3]))) {
		int camera_id = atoi(argv[3]);
		if (camera_id >= 0){ // from live cam
			cout << "from live cam" << endl;
		}
		else { // from ROS topic
			read_from_topic = true;
			if (argc > 4){
				image_topic = std::string(argv[4]); //argv[4] is topic name
			}
			printf("Reading images from topic %s\n", image_topic.c_str());
		}
	}
	else { // from disk
		cout << "read from disk" << endl;
	}


	if (argc >= 5) {
		all_pts_pub_gap = atoi(argv[4]);
	}
	printf("all_pts_pub_gap: %d\n", all_pts_pub_gap); // 0 ?????????????
	return 1;
}




