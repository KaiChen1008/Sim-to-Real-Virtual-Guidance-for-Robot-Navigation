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

#include <time.h>

clock_t start, end;
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

void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps);
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
	// 20200118
	start = clock();
	ros::init(argc, argv, "Monopub");
	ros::start();
	if (!parseParams(argc, argv)) {
		return EXIT_FAILURE;
	}
	// int n_images = vstrImageFilenames.size();

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true, true);
	ros::NodeHandle nodeHandler;
	ros::Publisher 	pub_pts_and_pose 	= nodeHandler.advertise<geometry_msgs::PoseArray>("pts_and_pose", 10);
	ros::Publisher 	pub_all_kf_and_pts 	= nodeHandler.advertise<geometry_msgs::PoseArray>("all_kf_and_pts", 10);
	ros::Publisher 	pub_pose			= nodeHandler.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 100);

	if (read_from_topic) 
	{
		ImageGrabber igb(SLAM, pub_pts_and_pose, pub_all_kf_and_pts, pub_pose);
		// subscribe topic
		ros::Subscriber sub = nodeHandler.subscribe(image_topic, 1000, &ImageGrabber::GrabImage, &igb);
		//ros::Subscriber sub = nodeHandler.subscribe(image_topic, 1, &ImageGrabber::GrabImage, &igb);
		ros::spin();
	}
	else{
		printf("Topic does not exist");
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

void publish(ORB_SLAM2::System &SLAM, ros::Publisher &pub_pts_and_pose, ros::Publisher &pub_all_kf_and_pts, ros::Publisher &pub_pose, int frame_id) {
	// all_pts_pub_gap = 0
	if (all_pts_pub_gap > 0 && pub_count >= all_pts_pub_gap) {
		pub_all_pts = true;
		pub_count = 0;
	}
	// cout << "is_keyframe                     : " << SLAM.getTracker()->mCurrentFrame.is_keyframe << endl;
	// cout << "getLoopClosing()->loop_detected : " << SLAM.getLoopClosing()->loop_detected << endl;
	// cout << "SLAM.getTracker()->loop_detected: " << SLAM.getTracker()->loop_detected << endl;
	
	if (pub_all_pts || 	SLAM.getLoopClosing()->loop_detected || 
						SLAM.getTracker()->loop_detected) {
							
		pub_all_pts = SLAM.getTracker()->loop_detected = SLAM.getLoopClosing()->loop_detected = false;
		geometry_msgs::PoseArray kf_pt_array;

		vector<ORB_SLAM2::KeyFrame*> key_frames = SLAM.getMap()->GetAllKeyFrames();
		//! placeholder for number of keyframes
		kf_pt_array.poses.push_back(geometry_msgs::Pose());
		sort(key_frames.begin(), key_frames.end(), ORB_SLAM2::KeyFrame::lId);
		unsigned int n_kf = 0;
		for (auto key_frame : key_frames) {
			// pKF->SetPose(pKF->GetPose()*Two);

			if (key_frame->isBad())
				continue;

			cv::Mat R = key_frame->GetRotation().t();
			vector<float> q = ORB_SLAM2::Converter::toQuaternion(R);
			cv::Mat twc = key_frame->GetCameraCenter();
			geometry_msgs::Pose kf_pose;

			kf_pose.position.x = twc.at<float>(0);
			kf_pose.position.y = twc.at<float>(1);
			kf_pose.position.z = twc.at<float>(2);
			kf_pose.orientation.x = q[0];
			kf_pose.orientation.y = q[1];
			kf_pose.orientation.z = q[2];
			kf_pose.orientation.w = q[3];
			kf_pt_array.poses.push_back(kf_pose);

			unsigned int n_pts_id = kf_pt_array.poses.size();
			//! placeholder for number of points
			kf_pt_array.poses.push_back(geometry_msgs::Pose());
			std::set<ORB_SLAM2::MapPoint*> map_points = key_frame->GetMapPoints();
			unsigned int n_pts = 0;
			for (auto map_pt : map_points) {
				if (!map_pt || map_pt->isBad()) {
					//printf("Point %d is bad\n", pt_id);
					continue;
				}
				cv::Mat pt_pose = map_pt->GetWorldPos();
				if (pt_pose.empty()) {
					//printf("World position for point %d is empty\n", pt_id);
					continue;
				}
				geometry_msgs::Pose curr_pt;
				//printf("wp size: %d, %d\n", wp.rows, wp.cols);
				//pcl_cloud->push_back(pcl::PointXYZ(wp.at<float>(0), wp.at<float>(1), wp.at<float>(2)));
				curr_pt.position.x = pt_pose.at<float>(0);
				curr_pt.position.y = pt_pose.at<float>(1);
				curr_pt.position.z = pt_pose.at<float>(2);
				kf_pt_array.poses.push_back(curr_pt);
				++n_pts;
			}
			geometry_msgs::Pose n_pts_msg;
			n_pts_msg.position.x = n_pts_msg.position.y = n_pts_msg.position.z = n_pts;
			kf_pt_array.poses[n_pts_id] = n_pts_msg;
			++n_kf;
		}
		geometry_msgs::Pose n_kf_msg;
		n_kf_msg.position.x = n_kf_msg.position.y = n_kf_msg.position.z = n_kf;
		kf_pt_array.poses[0] = n_kf_msg;
		kf_pt_array.header.frame_id = "1";
		kf_pt_array.header.seq = frame_id + 1;

		printf("Publishing data for %u keyfranmes\n", n_kf);
		// 20200118
		//pub_all_kf_and_pts.publish(kf_pt_array);
		// need publish camera_pose
	}
	else if (SLAM.getTracker()->mCurrentFrame.is_keyframe) {
		cout << "in else if" << endl;
		++pub_count;
		SLAM.getTracker()->mCurrentFrame.is_keyframe = false;
		ORB_SLAM2::KeyFrame* pKF = SLAM.getTracker()->mCurrentFrame.mpReferenceKF;

		cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);


		vector<ORB_SLAM2::KeyFrame*> vpKFs = SLAM.getMap()->GetAllKeyFrames();
		sort(vpKFs.begin(), vpKFs.end(), ORB_SLAM2::KeyFrame::lId);

		// Transform all keyframes so that the first keyframe is at the origin.
		// After a loop closure the first keyframe might not be at the origin.
		cv::Mat Two = vpKFs[0]->GetPoseInverse();

		Trw = Trw*pKF->GetPose()*Two;
		cv::Mat lit = SLAM.getTracker()->mlRelativeFramePoses.back();
		cv::Mat Tcw = lit * Trw;
		cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
		cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

		vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);
		
		std::vector<ORB_SLAM2::MapPoint*> map_points = SLAM.GetTrackedMapPoints();
		int n_map_pts = map_points.size();

		geometry_msgs::PoseArray pt_array;
		geometry_msgs::Pose camera_pose;

		camera_pose.position.x = twc.at<float>(0);
		camera_pose.position.y = twc.at<float>(1);
		camera_pose.position.z = twc.at<float>(2);

		camera_pose.orientation.x = q[0];
		camera_pose.orientation.y = q[1];
		camera_pose.orientation.z = q[2];
		camera_pose.orientation.w = q[3];

		pt_array.poses.push_back(camera_pose);

		for (int pt_id = 1; pt_id <= n_map_pts; ++pt_id){
			if (!map_points[pt_id - 1] || map_points[pt_id - 1]->isBad()) {
				//printf("Point %d is bad\n", pt_id);
				continue;
			}
			cv::Mat wp = map_points[pt_id - 1]->GetWorldPos();

			if (wp.empty()) {
				//printf("World position for point %d is empty\n", pt_id);
				continue;
			}
			geometry_msgs::Pose curr_pt;

			curr_pt.position.x = wp.at<float>(0);
			curr_pt.position.y = wp.at<float>(1);
			curr_pt.position.z = wp.at<float>(2);
			pt_array.poses.push_back(curr_pt);
			//printf("Done getting map point %d\n", pt_id);
		}

		pt_array.header.frame_id = "map";
		pt_array.header.seq = frame_id + 1;
		// 20200108
		//pub_pts_and_pose.publish(pt_array);
		//pub_kf.publish(camera_pose);
		geometry_msgs::PoseWithCovarianceStamped pose_with_c_and_s;

		pose_with_c_and_s.header = pt_array.header;
		// pose_with_c_and_s.pose.pose = camera_pose;
		// convert orb-slam coordination to Rviz coordination
		pose_with_c_and_s.pose.pose.position.y = camera_pose.position.z;
		pose_with_c_and_s.pose.pose.position.x = camera_pose.position.x;
		pose_with_c_and_s.pose.pose.position.z = 0;//- camera_pose.position.y;

		pose_with_c_and_s.pose.pose.orientation.y = camera_pose.orientation.z;
		pose_with_c_and_s.pose.pose.orientation.x = camera_pose.orientation.x;
		pose_with_c_and_s.pose.pose.orientation.z = camera_pose.orientation.y;
		pose_with_c_and_s.pose.pose.orientation.w = camera_pose.orientation.w;

		//for (int i = 0; i < 36; i++)
			// pose_with_c_and_s.pose.covariance[i] = 0.0;

		 //pub_pose.publish(ppose_with_c_and_sose_with_c_and_s);
	}
}

inline bool isInteger(const std::string & s){
	if (s.empty() || ((!isdigit(s[0])) && (s[0] != '-') && (s[0] != '+'))) return false;
	char * p;
	strtol(s.c_str(), &p, 10);
	return (*p == 0);
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps){
	ifstream fTimes;
	string strPathTimeFile = strPathToSequence + "/times.txt";
	fTimes.open(strPathTimeFile.c_str());
	while (!fTimes.eof()){
		string s;
		getline(fTimes, s);
		if (!s.empty()){
			stringstream ss;
			ss << s;
			double t;
			ss >> t;
			vTimestamps.push_back(t);
		}
	}

	string strPrefixLeft = strPathToSequence + "/image_0/";

	const int nTimes = vTimestamps.size();
	vstrImageFilenames.resize(nTimes);

	for (int i = 0; i < nTimes; i++)
	{
		stringstream ss;
		ss << setfill('0') << setw(6) << i;
		vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
	}
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
	// 20200108
	//std:: cout << "time: " << ((double)(clock()-start))/ CLOCKS_PER_SEC << std::endl;
	//start = clock();

	cv::Mat mTcw = SLAM.TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());
	// cout << "SLAM TrackMonocular size is " << mTcw.size() << endl;
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
		pose_with_c_and_s.pose.pose.orientation.x = -camera_pose.orientation.x;
		pose_with_c_and_s.pose.pose.orientation.z = -camera_pose.orientation.y;
		pose_with_c_and_s.pose.pose.orientation.w = camera_pose.orientation.w;

		for (int i = 0; i < 36; i++)
			pose_with_c_and_s.pose.covariance[i] = 0.0;

		// cv_ptr->header : PoseWithCovarianceStamped needed
		publish_counter ++;
		if (publish_counter == 1){
			pub_pose.publish(pose_with_c_and_s);
			publish_counter = 0;
		}
	}


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




