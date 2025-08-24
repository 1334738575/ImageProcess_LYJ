#ifndef IMAGEPROCESS_LYJ_DEFINES_H
#define IMAGEPROCESS_LYJ_DEFINES_H

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <base/PreDefine.h>
#include <base/CameraModule.h>
#include <base/Pose.h>
#include <opencv2/line_descriptor.hpp>
#include "ImageCommon/FeatureGrid.h"


#ifdef WIN32
#ifdef _MSC_VER
#define IMAGEPROCESS_LYJ_API __declspec(dllexport)
#else
#define IMAGEPROCESS_LYJ_API
#endif
#else
#define IMAGEPROCESS_LYJ_API
#endif



namespace ImageProcess_LYJ
{
	struct ImageExtractData
	{
		int id = -1;
		cv::Mat img; //原图
		cv::Mat mask; //掩膜
		cv::Mat depths; //深度图
		cv::Mat normal; //法线图
		SLAM_LYJ::CameraModule* cam = nullptr; //相机
		SLAM_LYJ::Pose3D Tcw; //相机位姿

		//keypoint
		bool usePointFeature = true;
		//std::vector<Eigen::Vector2f> kps_;
		std::vector<cv::KeyPoint> kps_;
		cv::Mat descriptors_;
		//KeyPointIndexMode kpIndMode_ = GRID;
		//std::shared_ptr<SLAM_LYJ::SLAM_LYJ_MATH::KdTree2d> kdtree_ = nullptr;
		std::shared_ptr<SLAM_LYJ::SLAM_LYJ_MATH::Grid2Df> grid_ = nullptr;
		std::shared_ptr<FeatureGrid> featureGrid_ = nullptr;
		//line
		bool useLineFeature = false;
		std::vector<cv::Vec4f> vecLines_;
		std::vector<cv::line_descriptor::KeyLine> vecKeyLines_;
		cv::Mat lineDescriptors_;
		//edge
		bool useEdgeFeature = false;
		std::vector<Eigen::Vector2f> edges_;
		std::shared_ptr<SLAM_LYJ::SLAM_LYJ_MATH::KdTree<float, 2>> kdtree_ = nullptr;
		std::vector<Eigen::Vector2f> edgesDir_;
		bool isParallel_ = false;
		Eigen::Vector3f nParallel_ = Eigen::Vector3f::Zero();
		Eigen::Vector3f nParallelVar_ = Eigen::Vector3f::Zero();
		cv::Mat dxM;
		cv::Mat dyM;
		cv::Mat cannyM;
	};

	struct ImageMatchData
	{
		int id1 = -1;
		int id2 = -1;

		//keypoint
		bool usePointMatch = true;
		std::vector<int> match2to1P;
		std::vector<float> weightsP;
		//line
		bool useLineMatch = false;
		std::vector<int> match2to1L;
		std::vector<float> weightsL;
		//edge
		bool useEdgeMatch = false;
		std::vector<int> match2to1E;
		std::vector<float> weightsE;
		//patch
		bool usePatchMatch = false;
		std::vector<int> match2to1H;
		std::vector<float> weightsH;

		//debug, tmp
		std::string debugPath = "";
	};

	struct ImageTriangleData
	{
		SLAM_LYJ::Pose3D T21;
		std::vector<bool> bTris;
		std::vector<Eigen::Vector3d> Ps; //if justTri==true, Ps is in world, else in camera 1
		bool justTri = false;
	};

}

#endif//IMAGEPROCESS_LYJ_DEFINES_H