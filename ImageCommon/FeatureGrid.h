#ifndef IMAGEPROCESS_LYJ_FEATURE_GRID_H
#define IMAGEPROCESS_LYJ_FEATURE_GRID_H

#include <opencv2/opencv.hpp>
#include <Eigen/Core>

#ifdef WIN32
#ifdef _MSC_VER
#define FEATUREGRID_LYJ_API __declspec(dllexport)
#else
#define FEATUREGRID_LYJ_API
#endif
#else
#define FEATUREGRID_LYJ_API
#endif


namespace ImageProcess_LYJ
{
	class FEATUREGRID_LYJ_API FeatureGrid {
	public:
	    FeatureGrid() = delete;
	    //resolution=20
	    FeatureGrid(const int w, const int h, const int resolution, const std::vector<cv::KeyPoint>& features);
	
	    void getKeypointIdsAround(const Eigen::Vector3d& line, std::vector<size_t>& ids);
	
	    inline int& getResolution() {
	        return resolution;
	    }
	public:
	    int resolution;
	    int max_row;
	    int max_col;
	    std::unordered_map<int, std::vector<size_t>> grid;
	};
}


#endif