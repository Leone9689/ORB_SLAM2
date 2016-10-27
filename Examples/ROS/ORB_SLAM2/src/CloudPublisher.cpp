#include "CloudPublisher.hpp"
#include <sstream>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

namespace ROS_ORB_SLAM
{
CloudPublisher::CloudPublisher()
{
  //mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
  //mImagePub = mNH.advertise<sensor_msgs::Image>("ORB_SLAM2/Cloud",10,true);
  pcl_pub = mNH.advertise<sensor_msgs::PointCloud2> ("ORB_SLAM2/Cloud", 1);
}

/*void CloudPublisher::refresh()
{
  //std::cerr << std::endl << "refresh " << foo << std::endl;
  foo += 1;
  PublishCloud();
}
*/
/*cv::Mat CloudPublisher::DrawFrame()
{
  cv::Mat im;
  mIm.copyTo(im);
    
  cv::Mat imWithInfo;
  DrawTextInfo(im, 0, imWithInfo);

  return imWithInfo;
    //return im;
}

void CloudPublisher::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
  std::stringstream s;
  s << "WAITING FOR IMAGES. (Topic: /camera/image_raw) "<<foo;
  int baseline=0;
  cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

  imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
  im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
  imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
  cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);
}
*/
void CloudPublisher::PublishCloud(pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    output.header.frame_id = "/my_frame";  
    /*pcl::PointCloud<pcl::PointXYZ> cloud;  

    cloud.width = clouds.rows;  
    cloud.height = clouds.cols;  
    cloud.points.resize(cloud.width * cloud.height);  
    
    for (size_t i = 0; i < cloud.points.size(); ++i)  
    {  
      cloud.points[i].x = clouds.at<cv::Vec3f>(i,1)[0];  
      cloud.points[i].y = clouds.at<cv::Vec3f>(i,1)[1];  
      cloud.points[i].z = clouds.at<cv::Vec3f>(i,1)[2];  
    }
    */
    pcl::toROSMsg(cloud, output);  
    pcl_pub.publish(output); 
}
/*
void CloudPublisher::Update(const cv::Mat cv_ptr)
{
  cv_ptr.copyTo(mIm);
  //std::cerr << std::endl << "update " << foo << std::endl;
}
*/
} // namespace ROS_ORB_SLAM
