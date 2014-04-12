#ifndef THE_MAINGOAL_TOBOT
#define THE_MAINGOAL_TOBOT

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "Transformer.hpp"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class GoalMaker{

	public : 
	ros::Publisher _ecrivain;
	Transformer* _trans;
	move_base_msgs::MoveBaseGoal _goal;
	public :
	GoalMaker(ros::Publisher& e) : _ecrivain(e){};
	GoalMaker() {};
	
	void setPublisher(ros::Publisher& e){_ecrivain=e;}
	void callBack(const geometry_msgs::PointStampedConstPtr& msg);
	void move();
};

#endif
