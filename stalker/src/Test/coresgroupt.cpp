#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include "MainGraphic.hpp"
#include <boost/test/unit_test.hpp>
#include "CorrespGrouping.hpp"

BOOST_AUTO_TEST_CASE(trying)
{
	CorrespGrouping<pcl::PointXYZRGBA> cg(new ShapeLocal<pcl::PointXYZRGBA>("bob1"), new ShapeLocal<pcl::PointXYZRGBA>("bob2"));
	
}