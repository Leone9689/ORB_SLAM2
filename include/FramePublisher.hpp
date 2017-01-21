#ifndef FRAMEPUBLISHER_H
#define FRAMEPUBLISHER_H

#include<opencv2/core/core.hpp>
#include <sstream>
#include<sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "ros/ros.h"

namespace ROS_ORB_SLAM
{

class FramePublisher
{
public:
  FramePublisher();  
  void refresh();
  void Update(const cv::Mat cv_ptr);
  void PublishFrame(const cv::Mat& image);
protected:
  cv::Mat DrawFrame();
  void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);
  cv::Mat mIm;
  ros::NodeHandle mNH;
  ros::Publisher mImagePub;
  int foo;
  int bar;
};
} // namespace ROS_ORB_SLAM
#endif // FRAMEPUBLISHER_H
