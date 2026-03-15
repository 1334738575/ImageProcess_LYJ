#ifndef IMAGEPROCESS_LYJ_FUNDAMENTAL_ESTIMATOR_H
#define IMAGEPROCESS_LYJ_FUNDAMENTAL_ESTIMATOR_H

#include <common/RANSAC.h>
#include "ImageProcess_LYJ_Defines.h"



namespace ImageProcess_LYJ
{

	class FundamentalDecomposer
	{
	public:
		FundamentalDecomposer();
		~FundamentalDecomposer();

		/**
		 * @brief 从本质矩阵E分解RT，并通过三角化验证正确解
		 * @param E 本质矩阵（3x3）
		 * @param K1/K2 相机内参矩阵（3x3）
		 * @param pts1/pts2 匹配特征点（归一化坐标，需一一对应）
		 * @param R_out 输出旋转矩阵
		 * @param t_out 输出平移向量
		 * @return 是否分解成功
		 */
		static bool decomposeEssentialMatrix(
			const cv::Mat& E,
			const cv::Mat& K1, const cv::Mat& K2,
			const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2,
			Eigen::Matrix3d& R_out, Eigen::Vector3d& t_out);
		/**
		 * @brief 从基础矩阵F分解RT（先转E，再分解）
		 * @param F 基础矩阵（3x3）
		 * @param K1/K2 相机内参
		 * @param pts1/pts2 像素坐标匹配点
		 * @param R_out/t_out 输出RT
		 * @return 是否成功
		 */
		static bool decomposeFundamentalMatrix(
			const cv::Mat& F,
			const cv::Mat& K1, const cv::Mat& K2,
			const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2,
			Eigen::Matrix3d& R_out, Eigen::Vector3d& t_out);

	private:

	};


class RANSACFundamental : public COMMON_LYJ::RANSACWithInd<float, Eigen::Matrix3f>
{
public:
	RANSACFundamental(std::vector<cv::KeyPoint>* _kps1, std::vector<cv::KeyPoint>* _kps2, 
		const std::vector<Eigen::Vector2i>& _matches,
		float _errTh,
		const double _preInlineRatio, const int _minNum2Solve,
		const int _maxIterNum = INT32_MAX, const double _dstSampleRatioTh = 0.99, const double _minRatio = 0.6)
		:COMMON_LYJ::RANSACWithInd<float, Eigen::Matrix3f>(_preInlineRatio, _minNum2Solve, _maxIterNum, _dstSampleRatioTh, _minRatio), m_errTh(_errTh), m_kps1(_kps1), m_kps2(_kps2), m_matches(_matches)
	{
		m_funcCalErrs = std::bind(&RANSACFundamental::calErr, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
		m_funcCalModule = std::bind(&RANSACFundamental::calMdl, this, std::placeholders::_1, std::placeholders::_2);
	}
	RANSACFundamental(std::vector<cv::KeyPoint>* _kps1, std::vector<cv::KeyPoint>* _kps2,
		const std::vector<int>& _matches2to1,
		float _errTh,
		const double _preInlineRatio, const int _minNum2Solve,
		const int _maxIterNum = INT32_MAX, const double _dstSampleRatioTh = 0.99, const double _minRatio = 0.6)
		:COMMON_LYJ::RANSACWithInd<float, Eigen::Matrix3f>(_preInlineRatio, _minNum2Solve, _maxIterNum, _dstSampleRatioTh, _minRatio), m_errTh(_errTh), m_kps1(_kps1), m_kps2(_kps2)
	{
		m_matches.reserve(_matches2to1.size() / 4);
		for (int i = 0; i < _matches2to1.size(); ++i)
		{
			if (_matches2to1[i] == -1)
				continue;
			m_matches.push_back(Eigen::Vector2i(i, _matches2to1[i]));
		}
		m_funcCalErrs = std::bind(&RANSACFundamental::calErr, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
		m_funcCalModule = std::bind(&RANSACFundamental::calMdl, this, std::placeholders::_1, std::placeholders::_2);
	}
	~RANSACFundamental() {};

protected:

	int calErr(const Eigen::Matrix3f& _mdl, const std::vector<int>& _datas, std::vector<float>& _errs, std::vector<bool>& _bInls);
	bool calMdl(const std::vector<int>& _samples, Eigen::Matrix3f& _mdl);

protected:
	float m_errTh = 0.01;
	std::vector<cv::KeyPoint>* m_kps1=nullptr;
	std::vector<cv::KeyPoint>* m_kps2=nullptr;
	std::vector<Eigen::Vector2i> m_matches;
};

}





#endif