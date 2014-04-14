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
#include <image_transport/image_transport.h>
#include "cv_bridge/cv_bridge.h"
#include <sensor_msgs/image_encodings.h>

int flag=0;


bool compareImg(cv::Mat& m1, cv::Mat& m2){
	cv::MatND hist, hist2;
	int channels[] = { 0, 1 };
	int h_bins = 50; int s_bins = 60;
	float h_ranges[] = { 0, 256 };
	float s_ranges[] = { 0, 180 };
	const float* ranges[] = { h_ranges, s_ranges };
	int histSize[] = { h_bins, s_bins };
	
	cv::calcHist( &m1, 1, channels, cv::Mat(), hist, 2, histSize, ranges, true, false );
	cv::normalize( hist, hist, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );
	
	cv::calcHist( &m2, 1, channels, cv::Mat(), hist2, 2, histSize, ranges, true, false );
	cv::normalize( hist2, hist2, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );
	
	double base_base = compareHist( hist, hist2, CV_COMP_CORREL );
	std::cout<<"Result "<<base_base<<std::endl;
	if(base_base>0.8){
		return true;
	}
	else{
		return false;
	}
	
}


void callBack(const sensor_msgs::ImageConstPtr& msg, Handler3D *theHandler){
	if(theHandler->cloud->isOrganized()){
		cv::Mat matrice;cv::Mat matrice2;
		theHandler->toMat(matrice, theHandler->cloud);
	
		cv_bridge::CvImagePtr image_msg =cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);		
		cv::Mat img=image_msg->image;
	
		//Compare if the conversion is working
		BOOST_CHECK(compareImg(img,matrice));

		/*Compare if the trunkage is working*/
		cv::Rect rec(1,1,200,300);
		theHandler->tracking(&rec);
		theHandler->toMat(matrice2, theHandler->cloud_filtered);
		cv::Mat ROI = img(rec); //Decoupe l'image

		BOOST_CHECK(compareImg(ROI,matrice2));
		
		exit(0);
	}
}



BOOST_AUTO_TEST_CASE(trying)
{
	ros::init(boost::unit_test::framework::master_test_suite().argc, boost::unit_test::framework::master_test_suite().argv, "winSize");
	ros::NodeHandle my_node;
	ros::Rate loop_rate(10);
	
	Handler3D *theHandler = new Handler3D();
	ros::Subscriber scribe_cloud = my_node.subscribe<sensor_msgs::PointCloud2> ("camera/depth/points_xyzrgb", 1, &Handler3D::setCloud, theHandler);
	
	image_transport::ImageTransport it(my_node);
	image_transport::CameraSubscriber scribe_image = it.subscribeCamera("camera/rgb/image", 1, boost::bind(callBack, _1, theHandler));
	
	ros::Publisher ciel=my_node.advertise<sensor_msgs::PointCloud2>("/cloud_pruned", 1);
	ros::Publisher pilote=my_node.advertise<geometry_msgs::PointStamped>("/tracking3D", 1000);
	theHandler->ciel=ciel;
	theHandler->pilote=pilote;
	
	while(ros::ok()){
		ros::spinOnce();
	}

}
