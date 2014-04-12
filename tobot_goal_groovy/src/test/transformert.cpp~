#include <iostream>
#define BOOST_TEST_DYN_LINK
#include <time.h>
#include <cstdlib>
#define BOOST_TEST_MODULE MyTest
#include <boost/test/unit_test.hpp>
#include "Transformer.hpp"
/*********************

*************************/

BOOST_AUTO_TEST_CASE(trying)
{
	ros::init(boost::unit_test::framework::master_test_suite().argc, boost::unit_test::framework::master_test_suite().argv, "timer");
	ros::NodeHandle n;
	
	ros::Rate loop_rate(10);

	Transformer t;
	
	geometry_msgs::PointStamped p_start;
	p_start.header.frame_id="camera";
	t.getTransformer(p_start);
	BOOST_CHECK_EQUAL(p_start.point.x,0.28);
	BOOST_CHECK_EQUAL(p_start.point.y,0);
	BOOST_CHECK_EQUAL(p_start.point.z,0.14);
	
}
