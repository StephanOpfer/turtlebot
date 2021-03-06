#include <ros/console.h>

#include <rviz/display_context.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

#include "DonatelloNavGoal.h"

#define ROBOT "donatello"
#define TOPIC "move_base_simple/goal"
#define DESC "Nav Goal"

namespace turtlebot_rviz_plugin
{

DonatelloNavGoal::DonatelloNavGoal()
{
}

DonatelloNavGoal::~DonatelloNavGoal()
{
}

void DonatelloNavGoal::onInitialize()
{
	PoseTool::onInitialize();
	setName(DESC "(" ROBOT ")");
	pub = nh.advertise<geometry_msgs::PoseStamped>(ROBOT "/" TOPIC, 1);
}

// protected
void DonatelloNavGoal::onPoseSet(double x, double y, double theta) {
	std::string fixed_frame = context_->getFixedFrame().toStdString();
	geometry_msgs::PoseStamped pose;

	pose.header.frame_id = fixed_frame; 
	pose.header.stamp = ros::Time::now(); 
	pose.pose.position.x = x; 
	pose.pose.position.y = y; 

	tf::Quaternion quat; 
	quat.setRPY(0.0, 0.0, theta); 
	tf::quaternionTFToMsg(quat, 
	                      pose.pose.orientation); 

	ROS_INFO("Setting pose of " ROBOT "to : %.3f %.3f %.3f [frame=%s]", x, y, theta, fixed_frame.c_str(
));
	pub.publish(pose);
}

} // namepsace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(turtlebot_rviz_plugin::DonatelloNavGoal, rviz::Tool)

