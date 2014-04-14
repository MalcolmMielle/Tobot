#ifndef DHANDLER_HPP
#define DHANDLER_HPP
#include <cstdlib>
#include <string>
//ros includes
#include "ros/ros.h"

//openCV includes

#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/highgui/highgui.hpp"

#include "sensor_msgs/PointCloud2.h"
#include <geometry_msgs/PointStamped.h>
#include "pcl_ros/point_cloud.h"
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

class Handler3D{
	public : 
	int flag;
	ros::Publisher pilote;
	ros::Publisher ciel;
	ros::Time time_stamp;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered;	
	sensor_msgs::PointCloud2ConstPtr cloud_sensor;
	
	Handler3D() : flag(0), time_stamp(ros::Time::now()), cloud(new pcl::PointCloud<pcl::PointXYZRGBA>), cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>), cloud_sensor(){
		cloud->height=1;
	};
	~Handler3D(){/*delete cloud;*/}
	void setCloud(const sensor_msgs::PointCloud2ConstPtr& cloudy);
	void toMat(cv::Mat& result, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudin);
	void publish(pcl::PointXYZRGBA& pt);
	void publish(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cf);
	void tracking(cv::Rect *currBB);
	void filter_pt(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudin, std::string axis, int limits1, int limits2);
	void afficheCloud();
	void afficheCloudFiltre();
};

#endif
