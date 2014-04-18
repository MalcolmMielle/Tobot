#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include "MainGraphic.hpp"
#include "Shape3DLocal.hpp"
#include <boost/test/unit_test.hpp>
#include "CorrespGrouping.hpp"

BOOST_AUTO_TEST_CASE(trying)
{
	
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGBA>);
	
	pcl::io::loadPCDFile ("/home/ros/catkin_ws/src/Tobot/stalker/src/Test/milk.pcd", *cloud);
	pcl::io::loadPCDFile ("/home/ros/catkin_ws/src/Tobot/stalker/src/Test/milk.pcd", *cloud2);

	CorrespGrouping<pcl::PointXYZRGBA>* p = new CorrespGrouping<pcl::PointXYZRGBA>(new ShapeLocal<pcl::PointXYZRGBA>, new ShapeLocal<pcl::PointXYZRGBA>);
	
	Main<pcl::PointXYZRGBA> mg(cloud, cloud2);
	
	mg.setScene(cloud);
	mg.setObject(cloud);
	
	mg.doWork();
	
	Main<pcl::PointXYZRGBA> mg2(p);
	mg2.setScene(cloud);
	mg2.setObject(cloud);
	mg2.doWork();
	
	Main<pcl::PointXYZRGBA> mg3(cloud, cloud, new CorrespGrouping<pcl::PointXYZRGBA>(new ShapeLocal<pcl::PointXYZRGBA>, new ShapeLocal<pcl::PointXYZRGBA>));
	mg3.doWork();
}