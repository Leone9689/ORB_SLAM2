#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include"Converter.h"
#include "OdomPublisher.hpp"

namespace ORB_SLAM2
{

OdomPublisher::OdomPublisher()
{
  odom_pub = nh.advertise<nav_msgs::Odometry>("/miivii_vslam/odom", 10); 

  odom.header.stamp = ros::Time::now();;  
  odom.header.frame_id = "world";  
  odom.child_frame_id = "base_link";  
} 

void OdomPublisher::PublishOdom(cv::Mat &Tcw)
{            
  //Eigen::Vector3d tcwEigen;
  cv::Mat Twc = Tcw.inv();                          
  cv::Mat tcwMat = Twc.rowRange(0,3).col(3);        
  //cv::cv2eigen(tcwMat,tcwEigen);   

  Eigen::Matrix<double,3,3> eigMat =ORB_SLAM2::Converter::toMatrix3d(Twc); 
  Eigen::Quaterniond q(eigMat);                     
       
  //set the position  
  odom.pose.pose.position.x = tcwMat.at<float>(2);  
  odom.pose.pose.position.y = -tcwMat.at<float>(0);  
  odom.pose.pose.position.z = -tcwMat.at<float>(1);  
  odom.pose.pose.orientation.x = -q.z();  
  odom.pose.pose.orientation.y = q.x();  
  odom.pose.pose.orientation.z = -q.y();  
  odom.pose.pose.orientation.w = q.w();  

  //set the velocity  
  /*odom.twist.twist.linear.x = 0;  
  odom.twist.twist.linear.y = 0;  
  odom.twist.twist.linear.z = 0;  
  odom.twist.twist.angular.x = 0;  
  odom.twist.twist.angular.y = 0;  
  odom.twist.twist.angular.z = 0;  
  */
  //publish the message  
  odom_pub.publish(odom);  
}            


}  // namespace ROS_ORB_SLAM 


