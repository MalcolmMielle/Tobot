#ifndef PIPELINE_RECKON_H
#define PIPELINE_RECKON_H

#include <iostream>
#include <stdio.h>
#include <vector>
#include <deque>
#include "Shape3D.hpp"
#include "Shape3DLocal.hpp"

template <typename PointType>
class Pipeline{
	
	public : 
	Shape<PointType>* _object; //Don't need because it take a Shape* from main.
	Shape<PointType>* _scene; // Need to be initialise because it takes a Cloud in argument.
	
	public : 
	Pipeline(Shape<PointType>* p, Shape<PointType>* p2) : _object(p), _scene(p2){}; //FREE PROBLEM
	~Pipeline(){
		delete _object;
		delete _scene;
		std::cout << "delete";
	}
	virtual void doPipeline() = 0;
	virtual void init(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2){
		_object->set(cloud);
		_scene->set(cloud2);
	}
	virtual void setObject(Shape<PointType>* o){delete _object; _object=o;}
	virtual void setScene(Shape<PointType>* o){delete _scene; _scene=o;}
	virtual void setObject(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr obj);
	virtual void setScene(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
	virtual Shape<PointType>* getObject(){return _object;}
	virtual Shape<PointType>* getScene(){return _scene;}
};

template <typename T>
inline void Pipeline<T>::setScene(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
	this->_scene->update(cloud);
}

template <typename T>
inline void Pipeline<T>::setObject(pcl::PointCloud< pcl::PointXYZRGBA >::Ptr obj)
{
	this->_object->update(obj);
}



#endif
	
