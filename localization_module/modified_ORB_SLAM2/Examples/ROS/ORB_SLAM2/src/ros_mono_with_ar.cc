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
#include<opencv2/imgproc/imgproc.hpp>

#include"../../../include/System.h"

#include "MapPoint.h"
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <Converter.h>

// AR
#include"AR/ViewerAR.h"

ORB_SLAM2::ViewerAR viewerAR;
bool bRGB = true;

cv::Mat K;
cv::Mat DistCoef;


//! parameters
bool read_from_topic = false, read_from_camera = false;
std::string image_topic = "/camera/image_raw";
int all_pts_pub_gap = 0;

vector<string> vstrImageFilenames;
vector<double> vTimestamps;
cv::VideoCapture cap_obj;

bool pub_all_pts = false;
int pub_count = 0;
int publish_counter = 0;

inline bool isInteger(const std::string & s);
void publish(ORB_SLAM2::System &SLAM, ros::Publisher &pub_pts_and_pose, ros::Publisher &pub_all_kf_and_pts,  ros::Publisher &pub_pose,  int frame_id);
bool parseParams(int argc, char **argv);

//modified
class ImageGrabber{
public:
	ImageGrabber(ORB_SLAM2::System &_SLAM, ros::Publisher &_pub_pts_and_pose, ros::Publisher &_pub_all_kf_and_pts,  ros::Publisher &_pub_pose) :
		SLAM(_SLAM), pub_pts_and_pose(_pub_pts_and_pose), pub_all_kf_and_pts(_pub_all_kf_and_pts), 
		pub_pose(_pub_pose), frame_id(0){}

	void GrabImage(const sensor_msgs::ImageConstPtr& msg);

	ORB_SLAM2::System &SLAM;
	ros::Publisher &pub_pts_and_pose;
	ros::Publisher &pub_all_kf_and_pts;
	ros::Publisher &pub_pose;
	int frame_id;
};


using namespace std;

int main(int argc, char **argv){
	ros::init(argc, argv, "Monopub");
	ros::start();
	if (!parseParams(argc, argv)) {
		return EXIT_FAILURE;
	}
	// int n_images = vstrImageFilenames.size();

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true, true);

	viewerAR.SetSLAM(&SLAM);

	ros::NodeHandle nodeHandler;
	ros::Publisher 	pub_pts_and_pose 	= nodeHandler.advertise<geometry_msgs::PoseArray>("pts_and_pose", 1000);
	ros::Publisher 	pub_all_kf_and_pts 	= nodeHandler.advertise<geometry_msgs::PoseArray>("all_kf_and_pts", 1000);
	ros::Publisher 	pub_pose			= nodeHandler.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 100);

	if (read_from_topic) 
	{
		ImageGrabber igb(SLAM, pub_pts_and_pose, pub_all_kf_and_pts, pub_pose);
		ros::Subscriber sub = nodeHandler.subscribe(image_topic, 1, &ImageGrabber::GrabImage, &igb);

		cv::FileStorage fSettings(argv[2], cv::FileStorage::READ);
		bRGB = static_cast<bool>((int)fSettings["Camera.RGB"]);
		float fps = fSettings["Camera.fps"];
		viewerAR.SetFPS(fps);
		float fx = fSettings["Camera.fx"];
    	float fy = fSettings["Camera.fy"];
    	float cx = fSettings["Camera.cx"];
    	float cy = fSettings["Camera.cy"];
		viewerAR.SetCameraCalibration(fx,fy,cx,cy);

		K = cv::Mat::eye(3,3,CV_32F);
		K.at<float>(0,0) = fx;
		K.at<float>(1,1) = fy;
		K.at<float>(0,2) = cx;
		K.at<float>(1,2) = cy;
		DistCoef = cv::Mat::zeros(4,1,CV_32F);
		DistCoef.at<float>(0) = fSettings["Camera.k1"];
		DistCoef.at<float>(1) = fSettings["Camera.k2"];
		DistCoef.at<float>(2) = fSettings["Camera.p1"];
		DistCoef.at<float>(3) = fSettings["Camera.p2"];
		const float k3 = fSettings["Camera.k3"];
    	if(k3!=0)
    	{
        	DistCoef.resize(5);
        	DistCoef.at<float>(4) = k3;
    	}

    	thread tViewer = thread(&ORB_SLAM2::ViewerAR::Run,&viewerAR);
		ros::spin();
	}
	else
	{
		cout << "Topic does not exist" << endl;
	}

	mkdir("results", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	SLAM.getMap()->Save("results//map_pts_out.obj");
	SLAM.getMap()->SaveWithTimestamps("results//map_pts_and_keyframes.txt");
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
	cv::Mat mTcw = SLAM.TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());

	if (mTcw.size().width == 4 &&  mTcw.size().height == 4)
	{
		cv::Mat mRcw = mTcw.rowRange(0,3).colRange(0,3);
		cv::Mat mRwc = mRcw.t();
		cv::Mat mtcw = mTcw.rowRange(0,3).col(3);
		cv::Mat twc = -mRcw.t()*mtcw;

		vector<float> q = ORB_SLAM2::Converter::toQuaternion(mRwc);
		geometry_msgs::Pose camera_pose;
		camera_pose.position.x = twc.at<float>(0);
		camera_pose.position.y = twc.at<float>(1);
		camera_pose.position.z = twc.at<float>(2);

		camera_pose.orientation.x = q[0];
		camera_pose.orientation.y = q[1];
		camera_pose.orientation.z = q[2];
		camera_pose.orientation.w = q[3];

		geometry_msgs::PoseWithCovarianceStamped pose_with_c_and_s;

		pose_with_c_and_s.header.seq = frame_id + 1;
		pose_with_c_and_s.header.frame_id = "map";

		// pose_with_c_and_s.pose.pose = camera_pose;
		// convert orb-slam coordination to Rviz coordination
		pose_with_c_and_s.pose.pose.position.y = camera_pose.position.z;
		pose_with_c_and_s.pose.pose.position.x = camera_pose.position.x;
		pose_with_c_and_s.pose.pose.position.z = 0;//- camera_pose.position.y;
		pose_with_c_and_s.pose.pose.orientation.y = camera_pose.orientation.z;
		pose_with_c_and_s.pose.pose.orientation.x = camera_pose.orientation.x;
		pose_with_c_and_s.pose.pose.orientation.z = camera_pose.orientation.y;
		pose_with_c_and_s.pose.pose.orientation.w = camera_pose.orientation.w;

		for (int i = 0; i < 36; i++)
			pose_with_c_and_s.pose.covariance[i] = 0.0;

		// cv_ptr->header : PoseWithCovarianceStamped needed
		publish_counter ++;
		if (publish_counter == 5){
			pub_pose.publish(pose_with_c_and_s);
			publish_counter = 0;
		}
	}
	// tansport view to AR thread
	cv::Mat im = cv_ptr->image.clone();
	cv::Mat imu;
	int state = SLAM.GetTrackingState();
	// cout << "state = mpSLAM->GetTrackingState() : " << state << endl; // NOT_YET, no_image, OK
	vector<ORB_SLAM2::MapPoint*> vMPs = SLAM.GetTrackedMapPoints();
	vector<cv::KeyPoint> vKeys = SLAM.GetTrackedKeyPointsUn();

	cv::undistort(im,imu,K,DistCoef);
	viewerAR.SetImagePose(imu,mTcw,state,vKeys,vMPs);

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
			cout << "please read from ROS topic" << endl;
		}
		else { // from ROS topic
			read_from_topic = true;
			if (argc > 4){
				image_topic = std::string(argv[4]); //argv[4] is topic name
			}
			printf("Reading images from topic %s\n", image_topic.c_str());
		}

	}

	if (argc >= 5) {
		all_pts_pub_gap = atoi(argv[4]); // = 0
	}
	printf("all_pts_pub_gap: %d\n", all_pts_pub_gap); // 0
	return 1;
}




