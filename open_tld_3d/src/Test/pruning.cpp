#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include "handler3D.hpp"
#include <boost/test/unit_test.hpp>
#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/highgui/highgui.hpp"
#include <opencv/cv.h>


/*********************
500 000 tick per sec by experiment...>>
*************************/

BOOST_AUTO_TEST_CASE(trying)
{
	srand (time(NULL));
	ros::init(boost::unit_test::framework::master_test_suite().argc, boost::unit_test::framework::master_test_suite().argv, "pruning");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
	
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGBA>);
	Handler3D theHandler;
	theHandler.cloud=cloud;
	cv::Rect rect(1,1,2,2);
	ros::Publisher ciel=n.advertise<sensor_msgs::PointCloud2>("/cloud_pruned", 1);
	ros::Publisher pilote=n.advertise<geometry_msgs::PointStamped>("/tracking3D", 1000);
	theHandler.ciel=ciel;
	theHandler.pilote=pilote;
	
	// Fill in the cloud data
	cloud->width  = 5;
	cloud->height = 5;
	cloud->points.resize (cloud->width * cloud->height);
	int y=0;
	int yy=0;
	int yyy=0;
	for (size_t i = 0; i < cloud->points.size (); ++i)
	{
	cloud->points[i].x = y;
	cloud->points[i].y = yy;
	cloud->points[i].z = yyy;
	y++;
	yyy++;
		if(y==5){
			y=0;
			yy++;
		}
	}

	cloud->points[6].x=std::numeric_limits<double>::quiet_NaN();;

	std::cerr << "Cloud before filtering: " << std::endl;
	theHandler.afficheCloud();
	theHandler.tracking(&rect);
	
	
	std::cerr << "Cloud after filtering: " << std::endl;
	for (size_t i = 0; i < theHandler.cloud_filtered->points.size (); ++i)
	std::cerr << "    " << theHandler.cloud_filtered->points[i].x << " " 
		      << theHandler.cloud_filtered->points[i].y << " " 
		      << theHandler.cloud_filtered->points[i].z << std::endl;
	/*while(ros::ok()){
		theHandler.tracking(&rect);
		ros::spinOnce();
	}*/
	
	
}
