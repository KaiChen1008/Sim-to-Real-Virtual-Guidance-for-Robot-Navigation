// A ROS node that recieves the ROS topic "slam_pose", which utilizes the localization module to obtain the current pose of the robot, and converts it to the transformation 
// between the robot frames "map" and "base_footprint". As we eliminate the use of a LIDAR, which originally uses gives the transformation, this node helps maintain
// the stuctural integrity of the ROS tf tree.

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>

tf::Transform transform;
tf::Quaternion q;

void pose_callback(const geometry_msgs::PoseWithCovarianceStampedPtr &pose)
{
   static tf::TransformBroadcaster br;
   q.setX(pose->pose.pose.orientation.x);                                       // Set orientation.x of transformation to the orientation.x of the pose given by "slam_pose" 
   q.setY(pose->pose.pose.orientation.y);                                       // Set orientation.y of transformation to the orientation.y of the pose given by "slam_pose"
   q.setZ(pose->pose.pose.orientation.z);                                       // Set orientation.z of transformation to the orientation.z of the pose given by "slam_pose"
   q.setW(pose->pose.pose.orientation.w);                                       // Set orientation.w of transformation to the orientation.w of the pose given by "slam_pose"

   transform.setOrigin(tf::Vector3(pose->pose.pose.position.x, pose->pose.pose.position.y, 0.0)); // Set position.x and position.y of transformation to the position.x and position.y of the pose given by "slam_pose" and set position.z to 0.0
   transform.setRotation(q);

   br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_footprint")); // Send transform as the transform between frames "map" and "base_footprint"
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "pose_tf");                                            // Set up ROS node named "pose_tf"
   ros::NodeHandle n("~");
   ros::Subscriber pose_sub = n.subscribe("/slam_pose", 1, pose_callback);      // Set up ROS subscriber that subscribes to ROS topic named "slam_pose"
   ros::Rate loop_rate(100);                                                    // Set loop frequency to 100Hz 
   while (ros::ok())
   {
      ros::spinOnce();
      loop_rate.sleep();
   }
}
