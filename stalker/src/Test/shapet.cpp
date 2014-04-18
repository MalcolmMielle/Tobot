#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include "Shape3DLocal.hpp"
#include <boost/test/unit_test.hpp>


BOOST_AUTO_TEST_CASE(trying)
{
	ShapeLocal<pcl::PointXYZRGBA> st("test");
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::io::loadPCDFile ("/home/ros/catkin_ws/src/Tobot/stalker/src/Test/milk.pcd", *cloud);
	st.update(cloud);
	st.loadMesh("/home/ros/catkin_ws/src/Tobot/stalker/src/Test/milk.pcd");
	st.compute();
	st.update(cloud);	
	st.setRadius(3);
	st.setSamplingSize(5);
	BOOST_CHECK_EQUAL(st.getRadius(), 3);
	BOOST_CHECK_EQUAL(st.getSamplingSize(),5);
	

}