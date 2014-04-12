#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "my_tf_listener");

	ros::NodeHandle node;
	tf::TransformListener listener(ros::Duration(10));
	geometry_msgs::PointStamped ultrasonic_point;

	ultrasonic_point.point.x = 0.0;
	ultrasonic_point.point.y = 0.0;
	ultrasonic_point.point.z = 0.0;
	tf::StampedTransform transform;
	ultrasonic_point.header.frame_id="camera";
	geometry_msgs::PointStamped bobo_point;
	
	try{
		geometry_msgs::PointStamped base_point;
		listener.waitForTransform("camera", "base_link", ros::Time(0), ros::Duration(1));

		 listener.transformPoint("base_link", ultrasonic_point, bobo_point);
	
		listener.lookupTransform("/camera", "/base_link", ros::Time(0), transform);
	}
	catch(tf::TransformException& ex){
		ROS_ERROR("Received an exception trying to transform a point from \"camera\" to \"base_link\": %s", ex.what());
	}
ROS_INFO("base_ultra: (%.2f, %.2f. %.2f) -----> ultra_link: (%.2f, %.2f, %.2f) at time %.2f",
        ultrasonic_point.point.x, ultrasonic_point.point.y, ultrasonic_point.point.z,
        bobo_point.point.x, bobo_point.point.y, bobo_point.point.z, bobo_point.header.stamp.toSec());
        
};
