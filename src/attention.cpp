#include <ros/ros.h>
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Float64.h"
#include "std_srvs/Empty.h"
#include <sstream>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "image_dif.hpp"
#include <boost/function.hpp> 

class ROIClipper{
  std::function<bool(cv::Vec3b)> predicate;
  ros::NodeHandle nh;
  ros::Subscriber sub_image;
  ros::Publisher pub_image;
  cv::Rect roi;
  bool isInit = true;

  public:
  ROIClipper(){
    pub_image = nh.advertise<sensor_msgs::Image>("/debug_image", 1);
    sub_image = nh.subscribe("/kinect_head/rgb/image_color", 1, &ROIClipper::callback, this);
    predicate = gen_hsi_filter(0.0, 1.0, 0.3, 1.0, 0.5, 0.88);
  }

  private:
  void callback(const sensor_msgs::Image& msg);

};

void ROIClipper::callback(const sensor_msgs::Image& msg)
{
  cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

  if(isInit){
    int mergin = 100;
    roi = determine_ROI(cv_ptr->image, predicate, mergin);
    isInit = false;
  }
  
  cv::Mat img_processed = convert_bf((cv_ptr->image)(roi), predicate);
  sensor_msgs::ImagePtr msg_debug = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_processed).toImageMsg();
  pub_image.publish(msg_debug);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "change_detector");
  ROIClipper rc;
  ros::spin();
  return 0;
}
