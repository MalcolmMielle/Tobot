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
	ros::init(boost::unit_test::framework::master_test_suite().argc, boost::unit_test::framework::master_test_suite().argv, "recep");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
	Handler3D theHandler;
	ros::Subscriber scribe_cloud = n.subscribe<sensor_msgs::PointCloud2> ("/cloud_pruned", 1, &Handler3D::setCloud, &theHandler);
	while(ros::ok){
		std::cout<<"Spin"<<std::endl;
		theHandler.afficheCloud();
		ros::spinOnce();
	}

}
