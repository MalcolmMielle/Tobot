#include <string.h>
#include "ros/ros.h"

#include "pcl_ros/point_cloud.h"
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

int main (int argc, char **argv){
	ros::init(argc, argv, "Stalker");
	ros::NodeHandle my_node;
	
	//Timer, Subscriber, Publisher description
	ros::Timer _imlost;
	ros::Subscriber pointcloud_sub;
	ros::Subscriber pc_filtered_sub;
	ros::Subscriber tracker2d_sub;
	ros::Publisher pose_pub;
	ros::Publisher newBB_pub;
	
	

	//scribe_cloud = my_node.subscribe<sensor_msgs::PointCloud2> ("camera/depth/points_xyzrgb", 1, &Handler3D::setCloud, theHandler);


	while(ros::ok()){
		ros::spinOnce();
	}

}
