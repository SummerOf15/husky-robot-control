#include "husky_highlevel_controller/HuskyHighlevelController.hpp"
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>

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
    publisher_ = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel",100); // delay will keep no message lost.
    publisher_marker=nodeHandle_.advertise<visualization_msgs::Marker>( "visualization_marker", 1);
    
  }

  HuskyHighlevelController::~HuskyHighlevelController()
  {
  }
  void HuskyHighlevelController::topicCallback(const sensor_msgs::LaserScan& message)
  {
    double krho, kalpha;
    if(!nodeHandle_.getParam("krho",krho))
    {
      ROS_ERROR("load krho failed");
      return;
    }
    if(!nodeHandle_.getParam("kalpha",kalpha))
    {
      ROS_ERROR("load kalpha failed");
      return;
    }

    float minVal = std::numeric_limits<float>::infinity();
    float angle=message.angle_max;
    for (unsigned int i = 0; i < message.ranges.size(); ++i)
    {
      if (std::isnormal(message.ranges[i]))
      {
        minVal = std::min(minVal, message.ranges[i]);
        angle=message.angle_min+message.angle_increment*i;
      }
    }
  
    ROS_INFO("distance: %f/%f, angle: %f",minVal,message.range_max,angle);

    //controller
    ros::Rate rate(10);
    geometry_msgs::Twist msg;
    double diff_angle=angle-0.0;
    msg.linear.x=krho*minVal;
    msg.linear.y=0;
    msg.linear.z=0;
    msg.angular.x=0;
    msg.angular.y=0.0;
    msg.angular.z=-kalpha*diff_angle;

    publisher_.publish(msg);
    rate.sleep();

    // // method 1: use rviz transform the frame
    // // publish marker information
    // visualization_msgs::Marker marker;
    // marker.header.frame_id=message.header.frame_id;
    // marker.header.stamp = ros::Time::now();
    // marker.ns = "my_namespace";
    // marker.id = 0;
    // marker.type = visualization_msgs::Marker::CYLINDER;
    // marker.action = visualization_msgs::Marker::ADD;
    // marker.pose.position.x = minVal*cos(angle);
    // marker.pose.position.y = minVal*sin(angle);
    // marker.pose.position.z = 0;
    // marker.pose.orientation.x = 0.0;
    // marker.pose.orientation.y = 0.0;
    // marker.pose.orientation.z = 0.0;
    // marker.pose.orientation.w = 1.0;
    // marker.scale.x = 0.5;
    // marker.scale.y = 0.5;
    // marker.scale.z = 1;
    // marker.color.a = 1.0; // Don't forget to set the alpha!
    // marker.color.r = 0.0;
    // marker.color.g = 1.0;
    // marker.color.b = 0.0;
    // marker.lifetime = ros::Duration();
    // publisher_marker.publish(marker);

    tf::StampedTransform transform;
    try
    {
      listener.lookupTransform("odom",message.header.frame_id, ros::Time(0),transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    
    double siny_cosp = 2 * (transform.getRotation().w() * transform.getRotation().z() + transform.getRotation().x() * transform.getRotation().y());
    double cosy_cosp = 1 - 2 * (transform.getRotation().y() * transform.getRotation().y() + transform.getRotation().z() * transform.getRotation().z());
    float trans_angle = atan2(siny_cosp, cosy_cosp);
    visualization_msgs::Marker marker;
    marker.header.frame_id="odom";
    marker.header.stamp = ros::Time::now();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = minVal*cos(trans_angle-angle)+transform.getOrigin().x();
    marker.pose.position.y = minVal*sin(trans_angle-angle)+transform.getOrigin().y();
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration();

    publisher_marker.publish(marker);
  }

} /* namespace */
