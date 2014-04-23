#ifndef CORRESGROUP_RECKON_H
#define CORRESGROUP_RECKON_H

#include "Pipeline.hpp"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/correspondence.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/features/board.h>
#include <pcl/features/shot_lrf.h>
#include <pcl/recognition/cg/geometric_consistency.h>

template <typename T>
class CorrespGrouping : public Pipeline<T> {
	protected : 
	pcl::CorrespondencesPtr _model_scene_corrs ;
	int _rf_rad; //Referance frame radius default 0.015
	int _cg_size; //Cluster size default 0.01
	int _cg_thresh; //Cluster thressold default 5
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > _rototranslations;
	std::vector<pcl::Correspondences> _clustered_corrs;
	public : 
	CorrespGrouping(Shape<T>* p, Shape<T>* p2) : Pipeline<T>(p, p2), _model_scene_corrs(new pcl::Correspondences ()), _rf_rad(0.015), _cg_size(0.01), _cg_thresh(5) {};
	
	virtual void doPipeline();
	
	//new stuff
	virtual void setFrameRadius(int rf){_rf_rad=rf;}
	virtual void setClusterSize(int rf){_cg_size=rf;}
	virtual void setClusterThresold(int rf){_cg_thresh=rf;}
	virtual int getFrameRadius(){return _rf_rad;}
	virtual int getClusterSize(){return _cg_size;}
	virtual int getClusterThresold(){return _cg_thresh;}
	
	virtual void point2PointCorrespondance();
	virtual void clusteringHough();

};

template <typename T>
inline void CorrespGrouping<T>::doPipeline()
{
	std::cout<<"getting in the pipeline"<<std::endl;

}

template <typename T>
inline void CorrespGrouping<T>::point2PointCorrespondance(){
	
	pcl::KdTreeFLANN<pcl::SHOT352> match_search;
	match_search.setInputCloud (this->_object->getDescr());
	std::cout << "yo";
	//  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
	for (size_t i = 0; i < this->_scene->getDescr()->size(); ++i){
		std::vector<int> neigh_indices(1);
		std::vector<float> neigh_sqr_dists(1);
		//skipping NaNs
		if (!pcl_isfinite (this->_scene->getDescr()->at (i).descriptor[0])) {
			continue;
		}

		int found_neighs = match_search.nearestKSearch (this->_scene->getDescr()->at(i), 1, neigh_indices, neigh_sqr_dists);

		//  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
		if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) {
			pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			_model_scene_corrs->push_back (corr);
			std::cout << "yo";
			
		}
	}
	std::cout << "Correspondences found: " << _model_scene_corrs->size() << std::endl;
}


template <typename T>
inline void CorrespGrouping<T>::clusteringHough(){
	    //
	//  Compute (Keypoints) Reference Frames only for Hough
	
	pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
	pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

	//USED THAT IN PCL 1.7
	/*pcl::BOARDLocalReferenceFrameEstimation<T, NormalType, RFType> rf_est;
	rf_est.setFindHoles (true);
	rf_est.setRadiusSearch (_rf_rad);

	rf_est.setInputCloud (this->_object->getKeypoints());
	rf_est.setInputNormals (this->_object->getNormals());
	rf_est.setSearchSurface (this->_object->getCloud());
	rf_est.compute (*model_rf);

	rf_est.setInputCloud (this->_scene->getKeypoints());
	rf_est.setInputNormals (this->_scene->getNormals());
	rf_est.setSearchSurface (this->_scene->getCloud());
	rf_est.compute (*scene_rf); */
	
	/*In PCL 1.6
	pcl::SHOTLocalReferenceFrameEstimation<T, RFType> rf_est;
	rf_est.setRadiusSearch (_rf_rad);

	rf_est.setInputCloud (this->_object->getKeypoints());
	rf_est.setSearchSurface (this->_object->getCloud());
	rf_est.compute (*model_rf);

	rf_est.setInputCloud (this->_scene->getKeypoints());
	rf_est.setSearchSurface (this->_scene->getCloud());
	rf_est.compute (*scene_rf);

	//  Clustering
	pcl::Hough3DGrouping<T, T, RFType, RFType> clusterer; //undefined reference in .o
	clusterer.setHoughBinSize (_cg_size);
	clusterer.setHoughThreshold (_cg_thresh);
	clusterer.setUseInterpolation (true);
	clusterer.setUseDistanceWeight (false);

	clusterer.setInputCloud (this->_object->getKeypoints());
	clusterer.setInputRf (model_rf);
	clusterer.setSceneCloud (this->_scene->getKeypoints());
	clusterer.setSceneRf (scene_rf);
	clusterer.setModelSceneCorrespondences (_model_scene_corrs);

	/*clusterer.cluster (clustered_corrs);
	clusterer.recognize (_rototranslations, _clustered_corrs);
	
	pcl::GeometricConsistencyGrouping<T, T> gc_clusterer;
	gc_clusterer.setGCSize (_cg_size);
	gc_clusterer.setGCThreshold (_cg_thresh);

	gc_clusterer.setInputCloud (this->_object->getKeypoints());
	gc_clusterer.setSceneCloud (this->_scene->getKeypoints());
	gc_clusterer.setModelSceneCorrespondences (_model_scene_corrs);

	//gc_clusterer.cluster (clustered_corrs);
	gc_clusterer.recognize (_rototranslations, _clustered_corrs);*/

}

#endif
