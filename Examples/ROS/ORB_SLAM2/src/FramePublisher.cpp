#include "FramePublisher.hpp"
#include <sstream>
#include<sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

namespace ROS_ORB_SLAM
{
FramePublisher::FramePublisher()
{
  //mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
  mImagePub = mNH.advertise<sensor_msgs::Image>("ORB_SLAM2/Frame",10,true);
  foo = 0;
  bar=0;
  //PublishFrame();
}

void FramePublisher::refresh()
{
  //std::cerr << std::endl << "refresh " << foo << std::endl;
  foo += 1;
  PublishFrame();
}

cv::Mat FramePublisher::DrawFrame()
{
  cv::Mat im;
  mIm.copyTo(im);
    
  cv::Mat imWithInfo;
  DrawTextInfo(im, 0, imWithInfo);

  return imWithInfo;
    //return im;
}

void FramePublisher::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
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

void FramePublisher::PublishFrame(cv::Mat &im)
{
    //cv::Mat im = DrawFrame();
    cv_bridge::CvImage rosImage;
    rosImage.image = im.clone();
    rosImage.header.stamp = ros::Time::now();
    rosImage.encoding = "bgr8";

    mImagePub.publish(rosImage.toImageMsg());
    ros::spinOnce();
}

void FramePublisher::Update(const cv::Mat cv_ptr)
{
  cv_ptr.copyTo(mIm);
  //std::cerr << std::endl << "update " << foo << std::endl;
}

} // namespace ROS_ORB_SLAM
