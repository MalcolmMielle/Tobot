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
	std::deque<Shape<PointType> *> _objects; //Deck of all objects to compare
	std::deque<Shape<PointType> *> _scenes; //Deck of all scene to compare
	
	public : 
	Pipeline(Shape<PointType>* p, Shape<PointType>* p2) : _object(p), _scene(p2){}; //FREE PROBLEM
	~Pipeline(){
		delete _object;
		delete _scene;
		clearObjects();
		clearScenes();
		std::cout << "delete Pipeline"<<std::endl;
	}
	virtual void doPipeline() = 0;
	virtual void init(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2){
		_object->set(cloud);
		_scene->set(cloud2);
	}
	virtual void setObject(Shape<PointType>* o){delete _object; _object=o;}
	virtual void setScene(Shape<PointType>* o){delete _scene; _scene=o;}
	virtual void setObject(typename pcl::PointCloud<PointType>::Ptr obj);
	virtual void setScene(typename pcl::PointCloud<PointType>::Ptr cloud);	
	virtual Shape<PointType>* getObject(){return _object;}
	virtual Shape<PointType>* getScene(){return _scene;}
	virtual void deleteObject(){delete _object;}
	virtual void deleteScene(){delete _scene;}
	
	//New interface : 
	virtual const std::deque<Shape<PointType> *>& getAllObjects(){return _objects;}
	virtual const std::deque<Shape<PointType> *>& getAllScenes(){return _scenes;}
	
	virtual void addObject(Shape<PointType>* o);
	virtual void addScene(Shape<PointType>* o);
	virtual void addObject(typename pcl::PointCloud<PointType>::Ptr o);
	virtual void addScene(typename pcl::PointCloud<PointType>::Ptr o);
	
	virtual void removeObject(const std::string& name);
	virtual void removeScene(const std::string& name);
	virtual void removeObject(Shape<PointType>* o);
	virtual void removeScene(Shape<PointType>* o);
	virtual void clearObjects();
	virtual void clearScenes();
	
};

template <typename T>
inline void Pipeline<T>::setScene(typename pcl::PointCloud<T>::Ptr cloud)
{
	this->_scene->update(cloud);
}

template <typename T>
inline void Pipeline<T>::setObject(typename pcl::PointCloud<T>::Ptr obj)
{
	this->_object->update(obj);
}

template <typename PointType>
inline void Pipeline<PointType>::addObject(Shape<PointType>* o){
	_objects.push_front(o);
	
}
template <typename PointType>
inline void Pipeline<PointType>::addScene(Shape<PointType>* o){
	_scenes.push_front(o);
}

template <typename PointType>
inline void Pipeline<PointType>::addObject(typename pcl::PointCloud<PointType>::Ptr o){
	_objects.push_front(new ShapeLocal<PointType>("newobj"));
	_objects[0]->update(o);
}
template <typename PointType>
inline void Pipeline<PointType>::addScene(typename pcl::PointCloud<PointType>::Ptr o){
	std::string name="newsce";
	name=name+boost::lexical_cast<string>( _scenes.size() );
	_scenes.push_front(new ShapeLocal<PointType>(name));
	_scenes[0]->update(o);
}

template <typename PointType>
inline void Pipeline<PointType>::removeObject(const std::string& name){
	for (typename std::deque<Shape<PointType>*>::iterator it=_objects.begin(); it!=_objects.end();){
		if((**it).getName()==name){
			delete(*it);
			_objects.erase(it);
		}
		else{
			it++;
		}
	}	
}

template <typename PointType>
inline void Pipeline<PointType>::removeScene(const std::string& name){
	for (typename std::deque<Shape<PointType>*>::iterator it = _scenes.begin(); it!=_scenes.end();){
		if((**it).getName()==name){
			delete(*it);
			_scenes.erase(it);
		}
		else{
			it++;
		}
	}	
}

template <typename PointType>
inline void Pipeline<PointType>::removeObject(Shape<PointType>* o){
	for (typename std::deque<Shape<PointType>*>::iterator it=_objects.begin(); it!=_objects.end();){
		if((*it)==o){
			delete(*it);
			_objects.erase(it);
		}
		else{
			it++;
		}
	}	
}

template <typename PointType>
inline void Pipeline<PointType>::removeScene(Shape<PointType>* o){
	for (typename std::deque<Shape<PointType>*>::iterator it=_scenes.begin(); it!=_scenes.end();){
		if((*it)==o){
			delete(*it);
			_scenes.erase(it);
		}
		else{
			it++;
		}
	}	
}

template <typename PointType>
inline void Pipeline<PointType>::clearObjects(){
	for(typename std::deque<Shape<PointType>*>::iterator it = _objects.begin(); it!=_objects.end();){
		delete(*it);
		_objects.erase(it);
	}	
}

template <typename PointType>
inline void Pipeline<PointType>::clearScenes(){
	for(typename std::deque<Shape<PointType>*>::iterator it = _scenes.begin(); it!=_scenes.end();){
		delete(*it);
		_scenes.erase(it);
	}	
}
#endif
	
