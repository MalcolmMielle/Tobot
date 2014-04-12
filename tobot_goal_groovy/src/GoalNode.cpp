#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Time.h"
#include "std_msgs/Duration.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sstream>
#include "GoalMaker.hpp"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>


int main(int argc, char **argv)
{
	//ROS declaration
	ROS_INFO("Starting the goal transmission node");
	ros::init(argc, argv, "GoalMaker");
	ros::NodeHandle my_node;
	ros::Rate loop_rate(10);
	
	Transformer* trans = new Transformer();
	//Publisher
	ros::Publisher ecrivain=my_node.advertise<geometry_msgs::PoseStamped>("destination", 1000);
	GoalMaker goalMaker(ecrivain);
	goalMaker._trans=trans;
	
	ros::Subscriber lecteur=my_node.subscribe<geometry_msgs::PointStamped> ("tracking3D", 1, &GoalMaker::callBack, &goalMaker);
	while (ros::ok()){
		ros::spinOnce();
	}
	
	delete trans;
	
}
