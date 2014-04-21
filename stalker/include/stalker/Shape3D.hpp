#ifndef SHAPE3D_MALCOLM_H
#define SHAPE3D_MALCOLM_H

#include <iostream>
#include <stdio.h>
#include <vector>
#include <deque>
#include "pcl_ros/point_cloud.h"
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/io/pcd_io.h>

#define DescriptorType pcl::SHOT352
#define NormalType pcl::Normal 
#define RFType pcl::ReferenceFrame 

template <typename T>
class Shape{

	protected :
	std::string _id;
	typename pcl::PointCloud<T>::Ptr _shape; //model
	typename pcl::PointCloud<T>::Ptr _shape_keypoints; //model_keypoint
	pcl::PointCloud<NormalType>::Ptr _shape_normals;	
	pcl::PointCloud<pcl::SHOT352>::Ptr _desc;	
	float _descrRad;
	float _shape_ss; //shape_sampling size
	float cg_size_ ; //clustering bin size
	float cg_thresh_ ;
	
	public :
	Shape(const std::string& name) : _id(name), _shape(new pcl::PointCloud<T>()),_shape_keypoints(new pcl::PointCloud<T>()),_shape_normals(new pcl::PointCloud<NormalType>()), _desc(new pcl::PointCloud<DescriptorType>()), _descrRad(0.02f), _shape_ss (0.01f), cg_size_ (0.01f),cg_thresh_ (5.0f){};

	virtual ~Shape(){
		std::cout<<"delete shape "<<_id <<std::endl;
	};
	//Accesseurs
	virtual const typename pcl::PointCloud<T>::Ptr& getCloud(){return _shape;}
	virtual const typename pcl::PointCloud<T>::Ptr& getCloudKey(){return _shape_keypoints;}
	virtual const typename pcl::PointCloud<NormalType>::Ptr& getNormals(){return _shape_normals;}
	virtual pcl::PointCloud<DescriptorType>::Ptr& getDescr(){return _desc;}
	virtual float getRadius(){return _descrRad;}
	virtual float getSamplingSize(){return _shape_ss;}
	virtual const std::string& getName(){return _id;}
	
	virtual void set(typename pcl::PointCloud<T>::Ptr& p){_shape=p;}
	virtual void setRadius(float r){_descrRad=r;}
	virtual void setSamplingSize(float r){_shape_ss=r;}
	
	//update Shape state
	
	virtual void compute()=0;

	//Load a model
	virtual void update(typename pcl::PointCloud<T>::Ptr& p);
	virtual bool loadMesh(std::string path);
	virtual bool saveMesh();

};

template <typename T>
inline void Shape<T>::update(typename pcl::PointCloud<T>::Ptr& p){
	//Clustering pipe
	this->_shape=p;
	compute();
}

template <typename T>
inline bool Shape<T>::saveMesh(){
	int a = pcl::io::savePCDFileBinary("view", *_shape);
	return true;
}

template <typename T>
inline bool Shape<T>::loadMesh(std::string path){
	if (pcl::io::loadPCDFile (path, *_shape) < 0)
	{
		std::cout << "Error loading model cloud." << std::endl;
		return false;
	}
	return true;
}

#endif
