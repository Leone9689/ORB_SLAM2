#ifndef MAPPUBLISHER_H
#define MAPPUBLISHER_H

#include<opencv2/core/core.hpp>
#include<ros/ros.h>
#include <visualization_msgs/Marker.h>

namespace ROS_ORB_SLAM
{
class MapPublisher
{
public:
  MapPublisher();
  void PublishCurrentCamera(const cv::Mat &Tcw);
private: 
  ros::NodeHandle nh;
  ros::Publisher publisher;
  float fCameraSize;
  visualization_msgs::Marker mCurrentCamera;
};
} // namespace ROS_ORB_SLAM

#endif // MAPPUBLISHER_H

