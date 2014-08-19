#include "handler3D.hpp"


void Handler3D::setCloud(const sensor_msgs::PointCloud2ConstPtr& cloudy){
	std::cout<<"HEllow"<<std::endl;
	cloud_sensor=cloudy;
	pcl::fromROSMsg(*cloudy, *cloud);
	flag==1;
}

void Handler3D::tracking(cv::Rect *currBB){
	if(currBB!=NULL && cloud->isOrganized()){
		//Point tracking
		double x_lim;
		double x_lim_max;
		double y_lim;
		double y_lim_max;
		
		if(cloud->isOrganized()){
			std::cout<<"Organized!"<<std::endl;
			int x=currBB->x+(currBB->width/2);
			int y=currBB->y+(currBB->height/2);
			pcl::PointXYZRGBA point(cloud->at(x,y));
			if(!isnan(point.x) && !isnan(point.y) && !isnan(point.z)){
				(*this).publish(point);
			}
		}
		
		cloud_filtered->width  = currBB->width;
		cloud_filtered->height = currBB->height;
		cloud_filtered->points.resize (cloud_filtered->width * cloud_filtered->height);
		int x_go=currBB->x;
		int y_go=currBB->y;
		int x_stop=currBB->width;
		int y_stop=currBB->height;
		int i=0;
		
		//TODO Make sure that the bounding box and the point cloud are in the same coordinate frame.
		
		for (size_t y = currBB->y; y < currBB->y+y_stop; ++y){
			for (size_t x = currBB->x; x < currBB->x+x_stop; ++x){
				/*cloud_filtered->points[i].x = cloud->points[x+(cloud->width)*y].x;
				cloud_filtered->points[i].y = cloud->points[x+(cloud->width)*y].y;
				cloud_filtered->points[i].z = cloud->points[x+(cloud->width)*y].z;*/
				i++;
				cloud_filtered->at(x-x_go,y-y_go)=cloud->at(x,y);	
			}
		}
		cloud_filtered->is_dense=cloud->is_dense;
		publish(cloud_filtered);
		cv::Mat mat;
		toMat(mat, cloud_filtered);
		imshow("chouette",mat);		
	}
}

void Handler3D::filter_pt(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudin, std::string axis, int limits1, int limits2){
	pcl::PassThrough<pcl::PointXYZRGBA> pass_x;
	pass_x.setInputCloud (cloudin);
	pass_x.setFilterFieldName (axis);
	pass_x.setFilterLimits (limits1, limits2);	
	//pass_x.setFilterLimitsNegative (true);
	pass_x.filter (*cloud_filtered);
}

void Handler3D::toMat(cv::Mat& result, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudin){

	if (cloudin->isOrganized()) {
	    result = cv::Mat(cloudin->height, cloudin->width, CV_8UC3);
	    
		  for (int h=0; h<result.rows; h++) {
		      for (int w=0; w<result.cols; w++) {
		          pcl::PointXYZRGBA point = cloudin->at(w, h);

		          Eigen::Vector3i rgb = point.getRGBVector3i();

		          result.at<cv::Vec3b>(h,w)[0] = rgb[2];
		          result.at<cv::Vec3b>(h,w)[1] = rgb[1];
		          result.at<cv::Vec3b>(h,w)[2] = rgb[0];
		      }
		  }
	}
	else{
		std::cerr << "Cloud not organized, can't apply the function" << std::endl;
	}
}

/*****************PUBLISH FUNCTIONS*******************/

void Handler3D::publish(pcl::PointXYZRGBA& point){

	geometry_msgs::PointStamped pt;
	pt.point.x=point.x;
	pt.point.y=point.y;
	pt.point.z=point.z;
	pt.header.stamp=ros::Time::now();
	pt.header.frame_id="camera_link";
	(*this).pilote.publish(pt);
}

void Handler3D::publish(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cf){

	sensor_msgs::PointCloud2 pc2;
	pcl::toROSMsg(*cf, pc2);
	
	pc2.header.stamp=ros::Time::now();
	pc2.header.frame_id=cloud_sensor->header.frame_id;
	(*this).ciel.publish(pc2);
}

/**********************PRINT********************/

void Handler3D::afficheCloud(){
	for (size_t i = 0; i < cloud->points.size (); ++i){
	if(!isnan(cloud->points[i].x)){
	std::cerr << "    " << cloud->points[i].x << " " 
	            << cloud->points[i].y << " " 
	            << cloud->points[i].z << std::endl;
	           }
	          }
}

void Handler3D::afficheCloudFiltre(){
	for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
	std::cerr << "    " << cloud_filtered->points[i].x << " " 
	            << cloud_filtered->points[i].y << " " 
	            << cloud_filtered->points[i].z << std::endl;
}

