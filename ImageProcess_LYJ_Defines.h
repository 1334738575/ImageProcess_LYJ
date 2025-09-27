#ifndef IMAGEPROCESS_LYJ_DEFINES_H
#define IMAGEPROCESS_LYJ_DEFINES_H

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <base/PreDefine.h>
#include <base/CameraModule.h>
#include <base/Pose.h>
#include <opencv2/line_descriptor.hpp>
#include "ImageCommon/FeatureGrid.h"
//#include <common/CommonAlgorithm.h>
#include <base/PreDefine.h>
#include <IO/SimpleIO.h>
#include <IO/MeshIO.h>
#include <DBoW3/Vocabulary.h>


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
	struct ImageTriangleOption
	{
		bool justTri = false;
	};
	struct ImageTriangleData
	{
		SLAM_LYJ::Pose3D T21;
		std::vector<bool> bTris;
		std::vector<Eigen::Vector3d> Ps; //if justTri==true, Ps is in world, else in camera 1
		bool justTri = false;
		int triSize = 0;
		bool triSuccess = false;
	};

	struct ImageMatchOption
	{
		bool bTriangle = true;
		bool usePointMatch = true;
		int pointMatchMode = 0;
		bool pointMatchCheck = true;
		float squareDThInMesh = 0.01f;

		bool useLineMatch = false;

		bool useEdgeMatch = false;

		bool usePatchMatch = false;
	};
	struct ImageMatchData
	{
		int id1 = -1;
		int id2 = -1;

		SLAM_LYJ::Pose3D T21;

		//keypoint
		bool usePointMatch = true;
		std::vector<int> match2to1P;
		std::vector<float> weightsP;
		int pointMatchSize = 0;
		ImageTriangleData triDatas;
		//line
		bool useLineMatch = false;
		std::vector<int> match2to1L;
		std::vector<float> weightsL;
		int lineMatchSize = 0;
		//edge
		bool useEdgeMatch = false;
		std::vector<int> match2to1E;
		std::vector<float> weightsE;
		int edgeMatchSize = 0;
		//patch
		bool usePatchMatch = false;
		std::vector<int> match2to1H;
		std::vector<float> weightsH;
		int patchMatchSize = 0;

		//debug, tmp
		std::string debugPath = "";
	};

	struct ImageExtractOption
	{
		bool usePointFeature = true;
		int pointExtractMode = 0;
		
		bool useLineFeature = false;
		
		bool useEdgeFeature = false;
	};
	struct ImageExtractData
	{
		int id = -1;
		std::string path = "";
		cv::Mat img; //原图
		cv::Mat mask; //掩膜
		cv::Mat depths; //深度图
		std::vector<uint32_t> fIds; //面片ID，大小与深度图相同
		SLAM_LYJ::CameraModule* cam = nullptr; //相机
		SLAM_LYJ::Pose3D Tcw; //相机位姿
		SLAM_LYJ::BaseTriMesh* btmPtr = nullptr;

		//keypoint
		bool usePointFeature = true;
		//std::vector<Eigen::Vector2f> kps_;
		std::vector<cv::KeyPoint> kps_;
		std::vector<Eigen::Vector3f> kp3Ds_;
		cv::Mat descriptors_;
		DBoW3::FeatureVector featureVec;
		DBoW3::BowVector bowVec;
		//KeyPointIndexMode kpIndMode_ = GRID;
		//std::shared_ptr<SLAM_LYJ::SLAM_LYJ_MATH::KdTree2d> kdtree_ = nullptr;
		std::shared_ptr<SLAM_LYJ::SLAM_LYJ_MATH::Grid2Df> grid_ = nullptr;
		std::shared_ptr<FeatureGrid> featureGrid_ = nullptr;
		std::shared_ptr<FeatureGridFromORB> featureGridFromORB_ = nullptr;
		//line
		bool useLineFeature = false;
		std::vector<cv::Vec4f> vecLines_;
		std::vector<SLAM_LYJ::Line2f> lines2f;//与vecLines对应
		std::vector<SLAM_LYJ::Line3f> line3Ds_;
		std::vector<cv::line_descriptor::KeyLine> vecKeyLines_;
		cv::Mat lineDescriptors_;
		//edge
		bool useEdgeFeature = false;
		std::vector<Eigen::Vector2f> edges_;
		std::vector<Eigen::Vector3f> edge3Ds_;
		std::vector<Eigen::Vector3f> edge3DNors_;
		std::vector<Eigen::Vector2f> edgesDir_;
		std::shared_ptr<SLAM_LYJ::SLAM_LYJ_MATH::KdTree<float, 2>> kdtree_ = nullptr;
		bool isParallel_ = false;
		Eigen::Vector3f nParallel_ = Eigen::Vector3f::Zero();
		Eigen::Vector3f nParallelVar_ = Eigen::Vector3f::Zero();
		cv::Mat dxM;
		cv::Mat dyM;
		cv::Mat cannyM;
	};

	IMAGEPROCESS_LYJ_API void site2Key(const uint32_t& _s1, const uint32_t& _s2, uint64_t& _k);
	IMAGEPROCESS_LYJ_API void key2Site(const uint64_t& _k, uint32_t& _s1, uint32_t& _s2);

}

#endif//IMAGEPROCESS_LYJ_DEFINES_H