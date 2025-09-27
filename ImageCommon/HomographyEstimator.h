#ifndef IMAGEPROCESS_LYJ_HOMOGRAPHY_ESTIMATOR_H
#define IMAGEPROCESS_LYJ_HOMOGRAPHY_ESTIMATOR_H

#include <common/RANSAC.h>
#include "ImageProcess_LYJ_Defines.h"



namespace ImageProcess_LYJ
{



class RANSACHomography : public SLAM_LYJ::SLAM_LYJ_MATH::RANSAC<int, float, Eigen::Matrix3f>
{
public:
	RANSACHomography(std::vector<cv::KeyPoint>* _kps1, std::vector<cv::KeyPoint>* _kps2, float _errTh,
		const double _inlineRatioTh, const int _minInlineNum,
		const int _maxIterNum = INT32_MAX, const double _dstInlineRatioTh = 0.99, const double _stopRatio = 1.0)
		:SLAM_LYJ::SLAM_LYJ_MATH::RANSAC<int, float, Eigen::Matrix3f>(_inlineRatioTh, _minInlineNum, _maxIterNum, _dstInlineRatioTh, _stopRatio), m_errTh(_errTh), m_kps1(_kps1), m_kps2(_kps2)
	{
	}
	~RANSACHomography() {};

protected:

	bool calErr(const Eigen::Matrix3f& mdl, const int& data, float& err);
	bool calMdl(const std::vector<const int*>& samples, Eigen::Matrix3f& mdl);

protected:
	double m_errTh = 0.01;
	std::vector<cv::KeyPoint>* m_kps1 = nullptr;
	std::vector<cv::KeyPoint>* m_kps2 = nullptr;

};

}





#endif