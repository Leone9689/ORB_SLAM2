#include "ros_viewer.hpp"

#include<ros/ros.h>
//#include <opencv2/core/eigen.hpp>
//#include <tf/transform_datatypes.h>
//#include <tf_conversions/tf_eigen.h>
//#include <Eigen/Core>
//#include <Eigen/Geometry>
//#include <Eigen/Eigen>

// can't find that in cv.... wtf!
void Mat2Quat(cv::Mat Mat, cv::Mat& Quat)
{
  //Determine Quaternions assuming Roation on the Matrix coming in, Mat
  float trace = Mat.at<float>(0,0) +Mat.at<float>(1,1) + Mat.at<float>(2,2);
  if( trace > 0 ) 
  {
    float s = 0.5f / sqrtf(trace+ 1.0f);
    Quat.at<float>(0) = 0.25f / s;
    Quat.at<float>(1)= ( Mat.at<float>(2,1) - Mat.at<float>(1,2) ) * s;
    Quat.at<float>(2) = ( Mat.at<float>(0,2) - Mat.at<float>(2,0) ) * s;
    Quat.at<float>(3) = ( Mat.at<float>(1,0) - Mat.at<float>(0,1) ) * s;
  } else {
    if ( Mat.at<float>(0,0) > Mat.at<float>(1,1) && Mat.at<float>(0,0) > Mat.at<float>(2,2) ) 
    {
      float s = 2.0f * sqrtf( 1.0f + Mat.at<float>(0,0) - Mat.at<float>(1,1) - Mat.at<float>(2,2));
      Quat.at<float>(0) = (Mat.at<float>(2,1) - Mat.at<float>(1,2) ) / s;
      Quat.at<float>(1) = 0.25f * s;
      Quat.at<float>(2) = (Mat.at<float>(0,1) + Mat.at<float>(1,0) ) / s;
      Quat.at<float>(3) = (Mat.at<float>(0,2) + Mat.at<float>(2,0) ) / s;
    } else if (Mat.at<float>(1,1) > Mat.at<float>(2,2)) {
      float s = 2.0f * sqrtf( 1.0f + Mat.at<float>(1,1) - Mat.at<float>(0,0) - Mat.at<float>(2,2));
      Quat.at<float>(0) = (Mat.at<float>(0,2) - Mat.at<float>(2,0) ) / s;
      Quat.at<float>(1) = (Mat.at<float>(0,1) + Mat.at<float>(1,0) ) / s;
      Quat.at<float>(2) = 0.25f * s;
      Quat.at<float>(3) = (Mat.at<float>(1,2) + Mat.at<float>(2,1) ) / s;
    } else {
      float s = 2.0f * sqrtf( 1.0f + Mat.at<float>(2,2) - Mat.at<float>(0,0) - Mat.at<float>(1,1) );
      Quat.at<float>(0) = (Mat.at<float>(1,0) - Mat.at<float>(0,1) ) / s;
      Quat.at<float>(1) = (Mat.at<float>(0,2) + Mat.at<float>(2,0) ) / s;
      Quat.at<float>(2) = (Mat.at<float>(1,2) + Mat.at<float>(2,1) ) / s;
      Quat.at<float>(3) = 0.25f * s;
    }
  }

}


namespace ORB_SLAM2 {

  RosViewer::RosViewer() {
	cout << "##in ros viewer constructor" << endl;
	mCameraPoseUpdated = false;
	mCameraPosePublisher = nh.advertise<geometry_msgs::PoseStamped>("ORB_SLAM2/camera/pose", 10);
	mMsgCameraPose.header.frame_id = "/slam";

	mCameraMarkerPublisher = nh.advertise<visualization_msgs::Marker>("ORB_SLAM2/camera/marker", 10);
	mMsgCameraMarker.header.frame_id = "/slam";
	//	mMsgCameraMarker.ns = CAMERA_NAMESPACE;
	mMsgCameraMarker.id=4;
	mMsgCameraMarker.type = visualization_msgs::Marker::LINE_LIST;
	mMsgCameraMarker.scale.x=0.001;//0.2; 0.03
	mMsgCameraMarker.pose.orientation.w=1.0;
	mMsgCameraMarker.action=visualization_msgs::Marker::ADD;
	mMsgCameraMarker.color.g=1.0f;
	mMsgCameraMarker.color.a = 1.0;
 }

  void RosViewer::Run() {
	cout << "##in ros viewer run" << endl;
	int fps = 10; ros::Rate r(fps);
	while (ros::ok()) {
	  PublishCamera();//igb._frame_pub.refresh();
	  r.sleep();
	}
  }
  
  void RosViewer::UpdateFrame(Tracking *pTracker) {
	
  }
  
  void RosViewer::SetCurrentCameraPose(const cv::Mat &Tcw) {
	unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
	mCameraPoseUpdated = true;
	//cout << "pose updated" << Tcw << endl;
  }
  
  void RosViewer::Register(System* pSystem) {
	cout << "####in RosViewer register" << endl;
	mpSystem = pSystem;
  }


  void RosViewer::PublishCamera() {
	cv::Mat Rwc(3,3,CV_32F);
	cv::Mat twc(3,1,CV_32F);
	cv::Mat foo(4,4,CV_32F);
	{
	  unique_lock<mutex> lock(mMutexCamera);
	  if (!mCameraPoseUpdated) return;
	  Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
	  twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
	  foo = mCameraPose.clone();
	  mCameraPoseUpdated = false;
	}
	PublishCameraPose(twc, Rwc);
	PublishCameraMarker(foo);
  }


  void  RosViewer::PublishCameraPose(cv::Mat twc, cv::Mat Rwc) {
	mMsgCameraPose.header.stamp = ros::Time::now();
    cv::Mat quat(4,1,CV_32F);
    Mat2Quat(Rwc, quat);

	mMsgCameraPose.pose.position.x = twc.at<float>(0,0);
	mMsgCameraPose.pose.position.y = twc.at<float>(1,0);
	mMsgCameraPose.pose.position.z = twc.at<float>(2,0);
	mMsgCameraPose.pose.orientation.x = quat.at<float>(0,0);
	mMsgCameraPose.pose.orientation.y = quat.at<float>(1,0);
	mMsgCameraPose.pose.orientation.z = quat.at<float>(2,0);
	mMsgCameraPose.pose.orientation.w = quat.at<float>(3,0);
	mCameraPosePublisher.publish(mMsgCameraPose);
	//cout << "pose displayed" << twc << endl;
  }

  
  void  RosViewer::PublishCameraMarker(cv::Mat foo) {
	mMsgCameraMarker.points.clear();
	float d = 0.04;
	const float h = d*0.8, z = d*0.5;
	//Camera is a pyramid. Define in camera coordinate system
	cv::Mat o = (cv::Mat_<float>(4,1) <<   0,  0, 0, 1);
	cv::Mat p1 = (cv::Mat_<float>(4,1) <<  d,  h, z, 1);
	cv::Mat p2 = (cv::Mat_<float>(4,1) <<  d, -h, z, 1);
	cv::Mat p3 = (cv::Mat_<float>(4,1) << -d, -h, z, 1);
	cv::Mat p4 = (cv::Mat_<float>(4,1) << -d,  h, z, 1);
	
	cv::Mat Twc = foo.inv();
	
	cv::Mat ow =  Twc*o;
	cv::Mat p1w = Twc*p1;
	cv::Mat p2w = Twc*p2;
	cv::Mat p3w = Twc*p3;
	cv::Mat p4w = Twc*p4;
	
	geometry_msgs::Point msgs_o,msgs_p1, msgs_p2, msgs_p3, msgs_p4;
	msgs_o.x=ow.at<float>(0);
	msgs_o.y=ow.at<float>(1);
	msgs_o.z=ow.at<float>(2);
	msgs_p1.x=p1w.at<float>(0);
	msgs_p1.y=p1w.at<float>(1);
	msgs_p1.z=p1w.at<float>(2);
	msgs_p2.x=p2w.at<float>(0);
	msgs_p2.y=p2w.at<float>(1);
	msgs_p2.z=p2w.at<float>(2);
	msgs_p3.x=p3w.at<float>(0);
	msgs_p3.y=p3w.at<float>(1);
	msgs_p3.z=p3w.at<float>(2);
	msgs_p4.x=p4w.at<float>(0);
	msgs_p4.y=p4w.at<float>(1);
	msgs_p4.z=p4w.at<float>(2);
	
	mMsgCameraMarker.points.push_back(msgs_o);
	mMsgCameraMarker.points.push_back(msgs_p1);
	mMsgCameraMarker.points.push_back(msgs_o);
	mMsgCameraMarker.points.push_back(msgs_p2);
	mMsgCameraMarker.points.push_back(msgs_o);
	mMsgCameraMarker.points.push_back(msgs_p3);
	mMsgCameraMarker.points.push_back(msgs_o);
	mMsgCameraMarker.points.push_back(msgs_p4);
	mMsgCameraMarker.points.push_back(msgs_p1);
	mMsgCameraMarker.points.push_back(msgs_p2);
	mMsgCameraMarker.points.push_back(msgs_p2);
	mMsgCameraMarker.points.push_back(msgs_p3);
	mMsgCameraMarker.points.push_back(msgs_p3);
	mMsgCameraMarker.points.push_back(msgs_p4);
	mMsgCameraMarker.points.push_back(msgs_p4);
	mMsgCameraMarker.points.push_back(msgs_p1);
	
	mMsgCameraMarker.header.stamp = ros::Time::now();
	
	mCameraMarkerPublisher.publish(mMsgCameraMarker);
  }



}
