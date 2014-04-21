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
	typename pcl::PointCloud<T>::Ptr _scene; //Last Scene receive
	typename pcl::PointCloud<T>::Ptr _object; //Last Object receive
	
	std::deque<typename pcl::PointCloud<T>::Ptr> _objects; //Beta List of all objects receive
	std::deque<typename pcl::PointCloud<T>::Ptr> _scenes; //Beta List of all scene receive
	
	/*I chose the point cloud because it's easier to have something general of use if we receive a Point Cloud through Ros nodes*/
	Pipeline<T>* _pipeline;
	
	public : 
	
	/********DEFAULT CONSTRCTOR***********/
	Main() : 
	_scene(new pcl::PointCloud<T>()), 
	_object(new pcl::PointCloud<T>()), 
	_pipeline(new CorrespGrouping<T>(new ShapeLocal<T>("object"), new ShapeLocal<T>("scene"))) 
	{
		std::cout<<"buiding the main awith nothing"<<std::endl;
	}

	/********you have the Clouds CONSTRCTOR***********/
	Main(typename pcl::PointCloud<T>::Ptr object, typename pcl::PointCloud<T>::Ptr scene) : 
	_scene(scene), 
	_object(object), 
	_pipeline(new CorrespGrouping<T>(new ShapeLocal<T>("object"), new ShapeLocal<T>("scene")))
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
		clearObjects();
		clearScenes();
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
	
	//New interface
	void addObject(typename pcl::PointCloud<T>::Ptr c){_objects.push_front(c);_pipeline->addObject(c);}
	void addScene(typename pcl::PointCloud<T>::Ptr c){_scenes.push_front(c);_pipeline->addScene(c);}
	void removeObject(typename pcl::PointCloud<T>::Ptr c);
	void removeScene(typename pcl::PointCloud<T>::Ptr c);
	void clearObjects();
	void clearScenes();
	const std::deque<typename pcl::PointCloud<T>::Ptr>& getAllObjects(){return _objects;}
	const std::deque<typename pcl::PointCloud<T>::Ptr>& getAllScenes(){return _scenes;}
	
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

template <typename T>
inline void Main<T>::removeObject(typename pcl::PointCloud<T>::Ptr c){
	for (typename std::deque<typename pcl::PointCloud<T>::Ptr>::iterator it = _objects.begin(); it!=_objects.end();){
		if((*it)==c){
			delete(*it);//I think this is wrong because it's not a pointer...
			_objects.erase(it);
		}
		else{
			it++;
		}
	}
}

template <typename T>
inline void Main<T>::removeScene(typename pcl::PointCloud<T>::Ptr c){
	for (typename std::deque<typename pcl::PointCloud<T>::Ptr>::iterator it = _scenes.begin(); it!=_scenes.end();){
		if((*it)==c){
			delete(*it);//Same as above
			_scenes.erase(it);
		}
		else{
			it++;
		}
	}
}

template <typename T>
inline void Main<T>::clearObjects(){
	for(typename std::deque<typename pcl::PointCloud<T>::Ptr>::iterator it = _objects.begin(); it!=_objects.end();){
		_objects.erase(it);
	}	
}

template <typename T>
inline void Main<T>::clearScenes(){
	for(typename std::deque<typename pcl::PointCloud<T>::Ptr>::iterator it = _scenes.begin(); it!=_scenes.end();){
		_scenes.erase(it);
	}
}

/**********************DO WORK FUNCTIONS***************/
template <typename T>
inline void Main<T>::doWork(const sensor_msgs::PointCloud2ConstPtr& cloudy){
	pcl::fromROSMsg(*cloudy, *_scene);
	addScene(_scene);
	_pipeline->setScene(_scene); 
	_pipeline->addScene(_scene); //Need to figure out how to change and know the shape's names !! Maybe a yaml file...
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
