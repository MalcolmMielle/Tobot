#ifndef RECKON_MAIN_H
#define RECKON_MAIN_H

#include <iostream>
#include <stdio.h>
#include "sensor_msgs/PointCloud2.h"

#include <Shape3DLocal.hpp>
#include <CorrespGrouping.hpp>
#include <Shape3D.hpp>
#include "Pipeline.hpp"


template <typename T>
class Main{
	protected : 
	typename pcl::PointCloud<T>::Ptr _scene; //Or a Shape here ? to solve the problem. The thing is as stuff is for now you'll need to free only one of the shape in Pipeline and it sucks. It's better if Main and Pipeline got exactly the same pointers so you can free all here. Just transform in shape in doWork.
	//Or maybe it's better to have a Point Cloud here and do all the transformation in between here and Pipeline and free there.
	//You'll see.
	
	typename pcl::PointCloud<T>::Ptr _object; //Maybe write a PointCloud here ?
	
	/*I chose the point cloud because it's easier to have something general of use if we receive a Point Cloud through Ros nodes*/
	Pipeline<T>* _pipeline;
	
	public : 
	
	/********DEFAULT CONSTRCTOR***********/
	Main() : 
	_scene(new pcl::PointCloud<T>()), 
	_object(new pcl::PointCloud<T>()), 
	_pipeline(new CorrespGrouping<T>(new ShapeLocal<T>(), new ShapeLocal<T>())) 
	{
		std::cout<<"buiding the main awith nothing"<<std::endl;
	}

	/********you have the Clouds CONSTRCTOR***********/
	Main(typename pcl::PointCloud<T>::Ptr object, typename pcl::PointCloud<T>::Ptr scene) : 
	_scene(scene), 
	_object(object), 
	_pipeline(new CorrespGrouping<T>(new ShapeLocal<T>(), new ShapeLocal<T>()))
	{
		std::cout<<"buiding the main"<<std::endl;
		initPipeline();
	}
	
	/********you have the Pipeline CONSTRCTOR***********/
	Main(Pipeline<T>* p) :
	_scene(new pcl::PointCloud<T>()), 
	_object(new pcl::PointCloud<T>()), 
	_pipeline(p)
	{
		std::cout<<"buiding the main with just a pipeline"<<std::endl;
	}

	/********You have everything CONSTRCTOR***********/
	Main(typename pcl::PointCloud<T>::Ptr object, typename pcl::PointCloud<T>::Ptr scene, Pipeline<T>* p) : 
	_scene(scene), 
	_object(object), 
	_pipeline(p)
	{
		std::cout<<"buiding the main"<<std::endl;
		initPipeline();
	}
	
	/*********************DESTRUCTOR*********************/
	virtual ~Main(){
		std::cout<<"deleting the main"<<std::endl;
		delete _pipeline;
	}

	
	/***********Init***********************/
	void init(typename pcl::PointCloud<T>::Ptr object, typename pcl::PointCloud<T>::Ptr scene){
		setScene(scene);
		setObject(object);
	}
	
	void initPipeline(typename pcl::PointCloud<T>::Ptr shape, typename pcl::PointCloud<T>::Ptr scene){
		//_pipeline->init(cloud, cloud2);
		_pipeline->setObject(shape);
		_pipeline->setScene(scene);
	}
	
	void initPipeline(){
		//_pipeline->init(cloud, cloud2);
		_pipeline->setObject(_object);
		_pipeline->setScene(_scene);
	}
	
	//Accesseur
	typename pcl::PointCloud<T>::Ptr getCloud(){return _scene;}
	typename pcl::PointCloud<T>::Ptr getShape(){return _object;}
	Pipeline<T>* getPipeline(){return _pipeline;}
	
	void setScene(typename pcl::PointCloud<T>::Ptr c);
	void setObject(typename pcl::PointCloud<T>::Ptr s);
	void setPipeline(Pipeline<T>* p){delete _pipeline; _pipeline=p;}

	
	virtual void doWork(const sensor_msgs::PointCloud2ConstPtr& cloudy); 
	virtual void doWork();
	
};
template <typename T>
inline void Main<T>::setScene(typename pcl::PointCloud<T>::Ptr c){
	this->_scene=c; 
	this->_pipeline->setScene(_scene);
}
template <typename T>
inline void Main<T>::setObject(typename pcl::PointCloud<T>::Ptr s){
	_object=s; 
	_pipeline->setObject(_object);
}


/**********************DO WORK FUNCTIONS***************/
template <typename T>
inline void Main<T>::doWork(const sensor_msgs::PointCloud2ConstPtr& cloudy){
	pcl::fromROSMsg(*cloudy, *_scene);
	_pipeline->setScene(_scene);
	doWork();
}

template <typename T>
inline void Main<T>::doWork(){
	std::cout<<"doWork"<<std::endl;
	//TODO processing here
	
	/**************************MAIN PIPELINE OF RECOGNITION**********************/
	_pipeline->doPipeline();
	
	//TODO Post Processing here

}


#endif
