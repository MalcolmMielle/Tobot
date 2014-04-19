#ifndef CORRESGROUP_RECKON_H
#define CORRESGROUP_RECKON_H

#include "Pipeline.hpp"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/correspondence.h>
//#include <pcl/recognition/cg/hough_3d.h>
//#include <pcl/recognition/cg/geometric_consistency.h>

template <typename T>
class CorrespGrouping : public Pipeline<T> {
	protected : 
	pcl::CorrespondencesPtr _model_scene_corrs ;
	public : 
	CorrespGrouping(Shape<T>* p, Shape<T>* p2) : Pipeline<T>(p, p2), _model_scene_corrs(new pcl::Correspondences ()) {};
	virtual void doPipeline();
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
	/*
	pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
	pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

	pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
	rf_est.setFindHoles (true);
	rf_est.setRadiusSearch (rf_rad_);

	rf_est.setInputCloud (model_keypoints);
	rf_est.setInputNormals (model_normals);
	rf_est.setSearchSurface (model);
	rf_est.compute (*model_rf);

	rf_est.setInputCloud (scene_keypoints);
	rf_est.setInputNormals (scene_normals);
	rf_est.setSearchSurface (scene);
	rf_est.compute (*scene_rf);

	//  Clustering
	pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
	clusterer.setHoughBinSize (cg_size_);
	clusterer.setHoughThreshold (cg_thresh_);
	clusterer.setUseInterpolation (true);
	clusterer.setUseDistanceWeight (false);

	clusterer.setInputCloud (model_keypoints);
	clusterer.setInputRf (model_rf);
	clusterer.setSceneCloud (scene_keypoints);
	clusterer.setSceneRf (scene_rf);
	clusterer.setModelSceneCorrespondences (model_scene_corrs);

	//clusterer.cluster (clustered_corrs);
	clusterer.recognize (rototranslations, clustered_corrs);*/
}

#endif
