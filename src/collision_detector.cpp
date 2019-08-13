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
#include <active_visual_perception/CostService.h>

class CollisionDetector{
  ros::NodeHandle nh;
  bool isInit;

  public:
  ChangeDetector();

  private:
  void init_common();

}


CollisionDetector::ChangeDetector(){
  service_init = nh.advertiseService("change_detector_init", &ChangeDetector::req_handler_init, this);
  init_common();
}

CollisionDetector::init_common(){
  isInit = true;
}



