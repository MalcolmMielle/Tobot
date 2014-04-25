#include "Transformer.hpp"

void Transformer::getTransformer(geometry_msgs::PointStamped p_start){
	std::string from=p_start.header.frame_id;
	try{
		_listener.waitForTransform(from, "base_link", ros::Time(0), ros::Duration(1));
		_listener.transformPoint("base_link", p_start, _endPoint);
	}
	catch(tf::TransformException& ex){
		ROS_ERROR("Received an exception trying to transform points: %s", ex.what());
		_endPoint.point.x=0;
		_endPoint.point.y=0;
		_endPoint.point.z=0;
	}
	
	try{
		_listener.waitForTransform(from, "base_link", ros::Time(0), ros::Duration(1));
		_listener.lookupTransform(from, "base_link", ros::Time(0), _transform);
	}
	catch(tf::TransformException& ex){
		ROS_ERROR("Received an exception trying to transform: %s", ex.what());
	}
}
