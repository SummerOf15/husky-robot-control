#include "husky_highlevel_controller/HuskyHighlevelController.hpp"

namespace husky_highlevel_controller
{
  HuskyHighlevelController::HuskyHighlevelController(ros::NodeHandle& nodeHandle) :
  nodeHandle_(nodeHandle)
  {
    std::string topic;
    if(!nodeHandle_.getParam("topic",topic))
    {
      ROS_ERROR("load topic failed");
      return;
    }
    int qs;
    if(!nodeHandle_.getParam("queensize",qs))
    {
      ROS_ERROR("load queensize failed");
      return;
    }
    subscriber_ = nodeHandle.subscribe(topic, qs,&HuskyHighlevelController::topicCallback, this);
  }

  HuskyHighlevelController::~HuskyHighlevelController()
  {
  }
  void HuskyHighlevelController::topicCallback(const sensor_msgs::LaserScan& message)
  {
    int n=sizeof(message.ranges)/sizeof(float);
    float minimum=1000.0;
    for (int i=0;i<n;i++)
    {
      if(!std::isnormal(message.ranges[i])|| message.ranges[i]<message.range_min ||message.ranges[i]<message.range_max)
      {
        continue;
      }
      if(minimum>message.ranges[i])
      {
        minimum=message.ranges[i];

      }
    }
    ROS_INFO_STREAM(minimum);
  }

} /* namespace */
