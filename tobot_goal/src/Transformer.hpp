#ifndef THE_TRANSFORMER_TOBOT
#define THE_TRANSFORMER_TOBOT

#include <ros/ros.h>
#include <tf/transform_listener.h>

class Transformer{
	protected : 
	tf::TransformListener _listener;
	geometry_msgs::PointStamped _endPoint;
	tf::StampedTransform _transform;
	public : 
	Transformer() : _listener(ros::Duration(10)){};
	
	void getTransformer(geometry_msgs::PointStamped p_start);
	
	geometry_msgs::PointStamped& getEnd(){return _endPoint;}
	tf::StampedTransform& getTransform(){return _transform;}
        
};

#endif
