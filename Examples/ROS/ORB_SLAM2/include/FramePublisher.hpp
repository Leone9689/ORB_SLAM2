#ifndef FRAMEPUBLISHER_H
#define FRAMEPUBLISHER_H

#include<opencv2/core/core.hpp>
#include "ros/ros.h"

namespace ROS_ORB_SLAM
{

class FramePublisher
{
public:
  FramePublisher();  
  void refresh();
  void Update(const cv::Mat cv_ptr);
protected:
  cv::Mat DrawFrame();
  void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);
  void PublishFrame(cv::Mat &im);
  cv::Mat mIm;
  ros::NodeHandle mNH;
  ros::Publisher mImagePub;
  int foo;
  int bar;
};
} // namespace ROS_ORB_SLAM
#endif // FRAMEPUBLISHER_H
