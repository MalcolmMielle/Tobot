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

	CorrespGrouping<pcl::PointXYZRGBA>* p = new CorrespGrouping<pcl::PointXYZRGBA>(new ShapeLocal<pcl::PointXYZRGBA>("object"), new ShapeLocal<pcl::PointXYZRGBA>("scene"));
	
	Main<pcl::PointXYZRGBA> mg(cloud, cloud2);//Mem lek
	
	mg.setScene(cloud);
	mg.setObject(cloud);
	
	mg.doWork();
	
	Main<pcl::PointXYZRGBA> mg2(p);
	mg2.setScene(cloud);
	mg2.setObject(cloud);
	mg2.doWork();
	
	Main<pcl::PointXYZRGBA> mg3(cloud, cloud, new CorrespGrouping<pcl::PointXYZRGBA>(new ShapeLocal<pcl::PointXYZRGBA>("object"), new ShapeLocal<pcl::PointXYZRGBA>("scene")));
	mg3.doWork();
	
	///////////////////////////////////////////////
	//MEMORY LEAKS SOMEWHERE
	Main<pcl::PointXYZRGBA> mainly; //Mem leak
	mainly.addObject(cloud);
	mainly.addScene(cloud);
	BOOST_CHECK_EQUAL(mainly.getAllObjects().size(),1);
	//mainly.addObject(cloud); <- Doesn't work because of the name...
	BOOST_CHECK_EQUAL(mainly.getPipeline()->getAllObjects().size(),1);
	BOOST_CHECK_EQUAL(mainly.getPipeline()->getAllScenes().size(),1);
	BOOST_CHECK_EQUAL(mainly.getAllScenes().size(),1);
	//Don't work
	//mainly.removeObject(cloud);
	//BOOST_CHECK_EQUAL(mainly.getAllObjects().size(),0);
	
	/////////////////////////////
	CorrespGrouping<pcl::PointXYZRGBA>* cp=new CorrespGrouping<pcl::PointXYZRGBA>(new ShapeLocal<pcl::PointXYZRGBA>("1"), new ShapeLocal<pcl::PointXYZRGBA>("2"));
	Main<pcl::PointXYZRGBA> meanie(cp);
};