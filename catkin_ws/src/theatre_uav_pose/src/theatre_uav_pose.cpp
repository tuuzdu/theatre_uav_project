#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

ros::Publisher pub_pose;

//void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg){
//  static tf::TransformBroadcaster br;
//  tf::Transform transform;
//  tf::poseMsgToTF(msg->pose.pose, transform);
////  tf::Quaternion q;
////  q.setRPY(3.14159265358979f, 0, 0);
////  q = transform.getRotation() * q;
////  transform.setRotation(q);
//  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/camera_link", "/base_link"));
//}

//void attitudeCallback(const sensor_msgs::ImuConstPtr& msg){
//  static tf::TransformBroadcaster br;
//  tf::Transform transform;
//
////  double r, p, y;
////  tf::Quaternion q;
////  tf::quaternionMsgToTF(msg->orientation,q);
////  tf::Matrix3x3(q).getRPY(r, p, y);
////  ROS_INFO("roll:%f\tpitch:%f\tyaw:%f\n", r*57.3, p*57.3, y*57.3);
//
//
//  transform.setRotation(tf::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w));
////  transform.setRotation(tf::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w) * tf::Quaternion(1,0,0,0));
////  transform.setRotation(transform.getRotation() * tf::Quaternion(0,0,1,0));
//
////  transform.setRotation(tf::Quaternion(msg->orientation.x, -msg->orientation.z, msg->orientation.w, msg->orientation.y));
//
//  transform.setOrigin(tf::Vector3(0,0,0.5));
//
//  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/camera_link", "/imu_link"));
//}

void odomCallback(const nav_msgs::OdometryConstPtr& msg){
 geometry_msgs::PoseStamped pose;
 pose.pose = msg->pose.pose;
 pose.header = msg->header;

 pub_pose.publish(pose);

}


int main(int argc, char** argv){
  ros::init(argc, argv, "theatre_uav_tf_broadcaster");

  ros::NodeHandle node;
//  ros::Subscriber sub_pose = node.subscribe("/monocular_pose_estimator/estimated_pose", 10, &poseCallback);

//  ros::Subscriber sub_att = node.subscribe("/mavros/imu/data", 10, &attitudeCallback);

  ros::Subscriber sub_odom = node.subscribe("/odometry/filtered", 10, &odomCallback);

  pub_pose = node.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1000);

  ros::spin();
  return 0;
};
