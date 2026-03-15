#ifndef IMAGEPROCESS_LYJ_HOMOGRAPHY_ESTIMATOR_H
#define IMAGEPROCESS_LYJ_HOMOGRAPHY_ESTIMATOR_H

#include <common/RANSAC.h>
#include "ImageProcess_LYJ_Defines.h"



namespace ImageProcess_LYJ
{
	class HomographyDecomposer
	{
	public:
		HomographyDecomposer();
		~HomographyDecomposer();

		/**
		 * @brief 从单应性矩阵H分解RT
		 * @param H 单应性矩阵（3x3，已归一化）
		 * @param K1/K2 相机内参
		 * @param pts1/pts2 像素匹配点
		 * @param R_out/t_out 输出RT
		 * @return 是否成功
		 */
		static bool decomposeHomographyMatrix(
			const cv::Mat& H,
			const cv::Mat& K1, const cv::Mat& K2,
			const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2,
			Eigen::Matrix3d& R_out, Eigen::Vector3d& t_out);
	private:

	};


class RANSACHomography : public COMMON_LYJ::RANSACWithInd<float, Eigen::Matrix3f>
{
public:
	RANSACHomography(std::vector<cv::KeyPoint>* _kps1, std::vector<cv::KeyPoint>* _kps2,
		const std::vector<Eigen::Vector2i>& _matches,
		float _errTh,
		const double _preInlineRatio, const int _minNum2Solve,
		const int _maxIterNum = INT32_MAX, const double _dstSampleRatioTh = 0.99, const double _minRatio = 0.6)
		:COMMON_LYJ::RANSACWithInd<float, Eigen::Matrix3f>(_preInlineRatio, _minNum2Solve, _maxIterNum, _dstSampleRatioTh, _minRatio), m_errTh(_errTh), m_kps1(_kps1), m_kps2(_kps2), m_matches(_matches)
	{
		m_funcCalErrs = std::bind(&RANSACHomography::calErr, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
		m_funcCalModule = std::bind(&RANSACHomography::calMdl, this, std::placeholders::_1, std::placeholders::_2);
	}
	~RANSACHomography() {};

protected:

	int calErr(const Eigen::Matrix3f& _mdl, const std::vector<int>& _datas, std::vector<float>& _errs, std::vector<bool>& _bInls);
	bool calMdl(const std::vector<int>& _samples, Eigen::Matrix3f& _mdl);

protected:
	float m_errTh = 0.01;
	std::vector<cv::KeyPoint>* m_kps1 = nullptr;
	std::vector<cv::KeyPoint>* m_kps2 = nullptr;
	std::vector<Eigen::Vector2i> m_matches;

};

}





#endif