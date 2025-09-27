#ifndef IMAGEPROCESS_LYJ_CORRESPOND_GRAPH_H
#define IMAGEPROCESS_LYJ_CORRESPOND_GRAPH_H


#include "ImageProcess_LYJ_Defines.h"


namespace ImageProcess_LYJ
{
	struct IMAGEPROCESS_LYJ_API CorrespondPoint
	{
		CorrespondPoint() {}
		CorrespondPoint(const uint32_t& _imageId, const uint32_t& _pointId)
			:imageId_(_imageId), pointId_(_pointId) 
		{}
		uint32_t imageId_ = UINT_MAX;
		uint32_t pointId_ = UINT_MAX;
	};
	struct IMAGEPROCESS_LYJ_API CorrespondImage
	{
		std::vector<std::vector<CorrespondPoint>> corrPoints_;
		std::vector<bool> bAdd2MapPoints;
		uint32_t corrNum_ = 0;
		SLAM_LYJ::SLAM_LYJ_MATH::CompressVV2V<CorrespondPoint> compressCorrPoints_;
		bool bCompressed_ = false;
		inline bool isCompressed() const{ return bCompressed_; }
		void compress();
		CorrespondPoint* getCorrespondPoint(const uint32_t& _pointId, int& _corrNum);
		const CorrespondPoint* getCorrespondPoint(const uint32_t& _pointId, int& _corrNum) const;
	};
	class IMAGEPROCESS_LYJ_API CorrespondGraph
	{
	public:
		CorrespondGraph();
		~CorrespondGraph();

		bool generate(std::map<uint32_t, std::shared_ptr<ImageExtractData>>& _imageDatas, std::unordered_map<uint64_t, std::shared_ptr<ImageProcess_LYJ::ImageMatchData>>& _matchDatas);
		bool compress();
		bool getData(std::map<uint32_t, std::shared_ptr<ImageExtractData>>& _imageDatas, std::unordered_map<uint64_t, std::shared_ptr<ImageProcess_LYJ::ImageMatchData>>& _matchDatas, SLAM_LYJ::PinholeCamera& _cam);

		bool existImage(const uint32_t& _id) const;
		bool existMatch(const uint32_t& _id1, const uint32_t& _id2) const;
		std::shared_ptr<ImageExtractData> getImage(const uint32_t& _id);
		const std::shared_ptr<ImageExtractData> getImage(const uint32_t& _id) const;
		std::shared_ptr<ImageMatchData> getMatches(const uint32_t& _id1, const uint32_t& _id2);
		const std::shared_ptr<ImageMatchData> getMatches(const uint32_t& _id1, const uint32_t& _id2) const;
		std::shared_ptr<CorrespondImage> getCorresponds(const uint32_t& _id);
		const std::shared_ptr<CorrespondImage> getCorresponds(const uint32_t& _id) const;
		CorrespondPoint* getCorrespondPoint(const uint32_t& _imageId, const uint32_t& _pointId, int& _corrNum);
		const CorrespondPoint* getCorrespondPoint(const uint32_t& _imageId, const uint32_t& _pointId, int& _corrNum) const;
		void getCorrespondPoint(const uint32_t& _imageId, const uint32_t& _pointId, std::vector<CorrespondPoint>& _corrPoints, int _turn=1) const;
		void setCorrPoint2MapPoint(const uint32_t& _imageId, const uint32_t& _pointId);
		bool isCorrPoint2MapPoint(const uint32_t& _imageId, const uint32_t& _pointId);
		//bool addCorrPoint(const uint32_t& _imgId1, const uint32_t& _pId1, const uint32_t& _imgId2, const uint32_t& _pId2);

		bool writeData(const std::string& _path, bool bCopyImg=false);
		bool readData(const std::string& _path);

	private:

	private:
		std::map<uint32_t, std::shared_ptr<ImageExtractData>> imageDatas_;
		std::unordered_map<uint64_t, std::shared_ptr<ImageMatchData>> matchDatas_;
		std::map<uint32_t, std::shared_ptr<CorrespondImage>> correspondDatas_;
		SLAM_LYJ::PinholeCamera cam_;
	};
}



#endif // !IMAGEPROCESS_LYJ_CORRESPOND_GRAPH_H
