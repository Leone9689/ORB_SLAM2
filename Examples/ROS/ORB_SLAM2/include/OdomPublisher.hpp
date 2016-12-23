#ifndef ODOMPUBLISHER_H
#define ODOMPUBLISHER_H

#include<vector>
#include<opencv2/core/core.hpp>
#include<ros/ros.h>
#include <nav_msgs/Odometry.h>
namespace ORB_SLAM2
{
class OdomPublisher
{
public:
  OdomPublisher();
  //void PublishMapPoints(const std::vector<MapPoint*> &vpMPs);
  void PublishOdom(cv::Mat &Tcw);
private: 
  ros::NodeHandle nh;
  ros::Publisher odom_pub; 
  nav_msgs::Odometry odom;  
};
} // namespace ROS_ORB_SLAM

#endif // MAPPUBLISHER_H

