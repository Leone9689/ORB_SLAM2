#include "MapPublisher.hpp"


namespace ORB_SLAM2
{

MapPublisher::MapPublisher()
{
  const char* MAP_FRAME_ID = "/world";
  //const char* MAP_FRAME_ID = "map";
  const char* CAMERA_NAMESPACE = "Camera";
  const char* POINTS_NAMESPACE = "MapPoints";    
 
  fPointSize=0.03;                                        
  mPoints.header.frame_id = MAP_FRAME_ID;                 
  mPoints.ns = POINTS_NAMESPACE;                          
  mPoints.id=0;                                           
  mPoints.type = visualization_msgs::Marker::POINTS;      
  mPoints.scale.x=fPointSize;                             
  mPoints.scale.y=fPointSize;                             
  mPoints.pose.orientation.w=1.0;                         
  mPoints.action=visualization_msgs::Marker::ADD;         
  mPoints.color.a = 1.0;                                  

  fCameraSize=0.4;
  mCurrentCamera.header.frame_id = MAP_FRAME_ID;
  mCurrentCamera.ns = CAMERA_NAMESPACE;
  mCurrentCamera.id=4;
  mCurrentCamera.type = visualization_msgs::Marker::LINE_LIST;
  mCurrentCamera.scale.x=0.03;//0.2; 0.03
  mCurrentCamera.pose.orientation.w=1.0;
  mCurrentCamera.action=visualization_msgs::Marker::ADD;
  mCurrentCamera.color.g=1.0f;
  mCurrentCamera.color.a = 1.0;

  publisher = nh.advertise<visualization_msgs::Marker>("/ORB_SLAM/Camera", 10);

  //publisher.publish(mPoints);             
  //publisher.publish(mCurrentCamera);

} 

void MapPublisher::PublishCurrentCamera(const cv::Mat &Tcw)
{
   mCurrentCamera.points.clear();
   float d = fCameraSize;
   const float h = d*0.8, z = d*0.5;
   //Camera is a pyramid. Define in camera coordinate system
   cv::Mat o = (cv::Mat_<float>(4,1) <<   0,  0, 0, 1);
   cv::Mat p1 = (cv::Mat_<float>(4,1) <<  d,  h, z, 1);
   cv::Mat p2 = (cv::Mat_<float>(4,1) <<  d, -h, z, 1);
   cv::Mat p3 = (cv::Mat_<float>(4,1) << -d, -h, z, 1);
   cv::Mat p4 = (cv::Mat_<float>(4,1) << -d,  h, z, 1);
   
   cv::Mat Twc = Tcw.inv();
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
   
   mCurrentCamera.points.push_back(msgs_o);
   mCurrentCamera.points.push_back(msgs_p1);
   mCurrentCamera.points.push_back(msgs_o);
   mCurrentCamera.points.push_back(msgs_p2);
   mCurrentCamera.points.push_back(msgs_o);
   mCurrentCamera.points.push_back(msgs_p3);
   mCurrentCamera.points.push_back(msgs_o);
   mCurrentCamera.points.push_back(msgs_p4);
   mCurrentCamera.points.push_back(msgs_p1);
   mCurrentCamera.points.push_back(msgs_p2);
   mCurrentCamera.points.push_back(msgs_p2);
   mCurrentCamera.points.push_back(msgs_p3);
   mCurrentCamera.points.push_back(msgs_p3);
   mCurrentCamera.points.push_back(msgs_p4);
   mCurrentCamera.points.push_back(msgs_p4);
   mCurrentCamera.points.push_back(msgs_p1);
   
   mCurrentCamera.header.stamp = ros::Time::now();
   
   publisher.publish(mCurrentCamera);
}

void MapPublisher::PublishMapPoints(cv::Mat &cloud)
{            
    mPoints.points.clear();
    for(int i=0;i<cloud.rows;i++)
    {
      geometry_msgs::Point p; 
      p.x =  cloud.at<cv::Vec3f>(i,0)[0];
      p.y =  cloud.at<cv::Vec3f>(i,0)[1];
      p.z =  cloud.at<cv::Vec3f>(i,0)[2];
      mPoints.points.push_back(p);
    }

  mPoints.header.stamp = ros::Time::now();
  publisher.publish(mPoints);
}            


}  // namespace ROS_ORB_SLAM 


