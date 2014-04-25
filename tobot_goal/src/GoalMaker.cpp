#include "GoalMaker.hpp"

void GoalMaker::callBack(const geometry_msgs::PointStampedConstPtr& msg){
	ROS_INFO("getting in the goalmaker");
	if(msg->header.stamp>ros::Time::now()-ros::Duration(0.01)){
		geometry_msgs::PointStamped endPoint;
	
		_trans->getTransformer(*msg);
		endPoint=_trans->getEnd();	
	
		//we'll send a goal to the robot
		_goal.target_pose.header.frame_id = "base_link";
		_goal.target_pose.header.stamp = ros::Time::now();
	
		if(endPoint.point.x>=0.1 || endPoint.point.x<=-0.1 || endPoint.point.y>=+0.1 || endPoint.point.y<=-0.1){
			_goal.target_pose.pose.position.x=endPoint.point.x;
			_goal.target_pose.pose.position.y=endPoint.point.y;
			_goal.target_pose.pose.position.z=0;
	
			//NEED TO DEFINE THE ORIENTATION BETTER THAN THAT AS WELL
			_goal.target_pose.pose.orientation.x=0;
			_goal.target_pose.pose.orientation.y=0;
			_goal.target_pose.pose.orientation.z=0;
			_goal.target_pose.pose.orientation.w=1;
			move();
		}
	}
}



void GoalMaker::move(){
	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	ROS_INFO("Sending goal");
	ac.sendGoal(_goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("Hooray, the base moved");
	else
		ROS_INFO("The base failed to move forward 1 meter for some reason");
}
