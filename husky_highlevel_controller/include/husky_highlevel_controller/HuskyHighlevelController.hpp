#pragma once

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

namespace husky_highlevel_controller {

/*!
 * Class containing the Husky Highlevel Controller
 */
class HuskyHighlevelController {
public:
	/*!
	 * Constructor.
	 */
	HuskyHighlevelController(ros::NodeHandle& nodeHandle);

	
	/*!
	 * Destructor.
	 */
	virtual ~HuskyHighlevelController();

private:
	void topicCallback(const sensor_msgs::LaserScan& message);


	ros::NodeHandle nodeHandle_;
	ros::Subscriber subscriber_;
	ros::Publisher publisher_;
	ros::Publisher publisher_marker;
	tf::TransformListener listener;
};

} /* namespace */
