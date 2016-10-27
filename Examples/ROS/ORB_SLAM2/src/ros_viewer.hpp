#ifndef ROSVIEWER_HPP
#define ROSVIEWER_HPP

#include "../../../include/Viewer.h"

#include<opencv2/core/core.hpp>
#include<ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

namespace ORB_SLAM2 {
  class Tracking;
  class System;


  class RosViewer : public ORB_SLAM2::Viewer {
  public:
	RosViewer();
	void Run();
	void UpdateFrame(Tracking *pTracker);
	void SetCurrentCameraPose(const cv::Mat &Tcw);
	void Register(System* pSystem);
  private:
	void PublishCamera();
	void PublishCameraPose(cv::Mat twc, cv::Mat Rwc);
	void PublishCameraMarker(cv::Mat foo);
	ros::NodeHandle nh;

	visualization_msgs::Marker mMsgCameraMarker;
	ros::Publisher mCameraMarkerPublisher;
	geometry_msgs::PoseStamped mMsgCameraPose;
	ros::Publisher mCameraPosePublisher;

	cv::Mat mCameraPose;
	bool mCameraPoseUpdated;
    std::mutex mMutexCamera;
	
  };
}


#endif // ROSVIEWER_HPP
