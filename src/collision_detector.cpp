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

class CollisionDetector{
  std::function<bool(cv::Vec3b)> predicate;
  ros::NodeHandle nh;
  ros::Subscriber sub_image;
  ros::Publisher pub_image, pub_cost, pub_cost_max, pub_cost_absolute;
  ros::ServiceServer service_init;
  cv::Rect roi;
  cv::Mat img_pre, img_init;
  bool isInit = true;
  int cost, cost_max, cost_absolute;

  public:
  void init_common();
  CollisionDetector(){
    pub_cost = nh.advertise<std_msgs::Int64>("/cost", 1);
    pub_cost_max = nh.advertise<std_msgs::Int64>("/cost_max", 1);
    pub_cost_absolute = nh.advertise<std_msgs::Int64>("/cost_absolute", 1);
    pub_image = nh.advertise<sensor_msgs::Image>("/debug_image", 1);
    sub_image = nh.subscribe("/kinect_head/rgb/image_color", 1, &CollisionDetector::callback, this);
    service_init = nh.advertiseService("image_collision_detection_init", &CollisionDetector::req_handler_init, this);
    predicate = gen_hsi_filter(0.0, 1.0, 0.3, 1.0, 0.5, 0.88);
    init_common();
  }

  private:
  void callback(const sensor_msgs::Image& msg);
  bool req_handler_init(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);//deprecated

};

void CollisionDetector::init_common()
{
  isInit = true;
  cost = 0;
  cost_max = 0;
  cost_absolute = 0;
}

void CollisionDetector::callback(const sensor_msgs::Image& msg)
{
  cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv::Mat img_processed;

  if(isInit){
    int mergin = 100;
    roi = determine_ROI(cv_ptr->image, predicate, mergin);
    img_processed = convert_bf((cv_ptr->image)(roi), predicate);
    img_pre = img_processed;
    img_init = img_pre;
    isInit = false;
    return;
  }

  img_processed = convert_bf((cv_ptr->image)(roi), predicate);
  cost = compute_cost(img_processed, img_pre);
  cost_absolute = compute_cost(img_processed, img_init);
  if(cost > cost_max){
    cost_max = cost;
  }
  std_msgs::Int64 msg_cost, msg_cost_max, msg_cost_absolute;
  msg_cost.data = cost;
  msg_cost_max.data = cost_max;
  msg_cost_absolute.data = cost_absolute;

  pub_cost.publish(msg_cost);
  pub_cost_max.publish(msg_cost_max);
  pub_cost_absolute.publish(msg_cost_absolute);
  sensor_msgs::ImagePtr msg_debug = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_processed).toImageMsg();
  pub_image.publish(msg_debug);
  img_pre = img_processed;
}

bool CollisionDetector::req_handler_init(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)//deprecated
{
  ROS_INFO("received; initialize change detector");
  init_common();
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "change_detector");
  CollisionDetector rc;
  ros::spin();
  return 0;
}
