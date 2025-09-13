#include "CorrespondGraph.h"
#include <thirdParty/STLPlus/include/file_system.h>

namespace ImageProcess_LYJ
{
	void CorrespondImage::compress()
	{
		compressCorrPoints_.compress(corrPoints_);
		corrNum_ = 0;
		for (auto& v : corrPoints_)
			corrNum_ += v.size();
		corrPoints_.clear();
		bCompressed_ = true;
	}
	CorrespondPoint* CorrespondImage::getCorrespondPoint(const uint32_t& _pointId, int& _corrNum)
	{
		_corrNum = 0;
		if (bCompressed_)
		{
			if (compressCorrPoints_.empty() || _pointId >= compressCorrPoints_.dataSize())
				return nullptr;
			_corrNum = compressCorrPoints_.subSize(_pointId);
			if (_corrNum == 0)
				return nullptr;
			return compressCorrPoints_.subDataPtr(_pointId);
		}
		else
		{
			if (corrPoints_.empty() || _pointId >= corrPoints_.size())
				return nullptr;
			_corrNum = corrPoints_[_pointId].size();
			if (_corrNum == 0)
				return nullptr;
			return corrPoints_[_pointId].data();
		}
		return nullptr;
	}
	const CorrespondPoint* CorrespondImage::getCorrespondPoint(const uint32_t& _pointId, int& _corrNum) const
	{
		_corrNum = 0;
		if (bCompressed_)
		{
			if (compressCorrPoints_.empty() || _pointId >= compressCorrPoints_.dataSize())
				return nullptr;
			_corrNum = compressCorrPoints_.subSize(_pointId);
			if (_corrNum == 0)
				return nullptr;
			return compressCorrPoints_.subDataPtr(_pointId);
		}
		else
		{
			if (corrPoints_.empty() || _pointId >= corrPoints_.size())
				return nullptr;
			_corrNum = corrPoints_[_pointId].size();
			if (_corrNum == 0)
				return nullptr;
			return corrPoints_[_pointId].data();
		}
		return nullptr;
	}

	CorrespondGraph::CorrespondGraph()
	{}
	CorrespondGraph::~CorrespondGraph()
	{}

	bool CorrespondGraph::generate(std::map<uint32_t, std::shared_ptr<ImageExtractData>>& _imageDatas, std::unordered_map<uint64_t, std::shared_ptr<ImageProcess_LYJ::ImageMatchData>>& _matchDatas)
	{
		correspondDatas_.clear();
		imageDatas_ = _imageDatas;
		matchDatas_ = _matchDatas;
		// 构建对应关系
		for (auto& mit : matchDatas_)
		{
			auto& mtd = mit.second;
			if (!mtd->usePointMatch || mtd->match2to1P.empty())
				continue;
			if (imageDatas_.find(mtd->id1) == imageDatas_.end() || imageDatas_.find(mtd->id2) == imageDatas_.end())
				continue;
			if (correspondDatas_.find(mtd->id1) == correspondDatas_.end())
			{
				correspondDatas_[mtd->id1] = std::make_shared<CorrespondImage>();
				correspondDatas_[mtd->id1]->corrPoints_.resize(imageDatas_[mtd->id1]->kps_.size());
			}
			if (correspondDatas_.find(mtd->id2) == correspondDatas_.end())
			{
				correspondDatas_[mtd->id2] = std::make_shared<CorrespondImage>();
				correspondDatas_[mtd->id2]->corrPoints_.resize(imageDatas_[mtd->id2]->kps_.size());
			}
			for (int i = 0; i < mtd->match2to1P.size(); ++i)
			{
				if (mtd->match2to1P[i] == -1)
					continue;
				correspondDatas_[mtd->id1]->corrPoints_[i].emplace_back(mtd->id2, mtd->match2to1P[i]);
				correspondDatas_[mtd->id2]->corrPoints_[mtd->match2to1P[i]].emplace_back(mtd->id1, i);
			}
		}
		for (auto& cit : correspondDatas_)
			cit.second->compress();
		SLAM_LYJ::PinholeCamera* camera = dynamic_cast<SLAM_LYJ::PinholeCamera*>(imageDatas_.begin()->second->cam);
		cam_ = *camera;
		return true;
	}
	bool CorrespondGraph::getData(std::map<uint32_t, std::shared_ptr<ImageExtractData>>& _imageDatas, std::unordered_map<uint64_t, std::shared_ptr<ImageProcess_LYJ::ImageMatchData>>& _matchDatas, SLAM_LYJ::PinholeCamera& _cam)
	{
		_imageDatas = imageDatas_;
		_matchDatas = matchDatas_;
		_cam = cam_;
		return true;
	}
	bool CorrespondGraph::existImage(const uint32_t& _id) const { return imageDatas_.count(_id) > 0; }
	bool CorrespondGraph::existMatch(const uint32_t& _id1, const uint32_t& _id2) const
	{
		uint64_t key;
		site2Key(_id1, _id2, key);
		return matchDatas_.count(key) > 0;
	}
	std::shared_ptr<ImageExtractData> CorrespondGraph::getImage(const uint32_t& _id)
	{
		if (imageDatas_.count(_id) == 0)
			return nullptr;
		return imageDatas_[_id];
	}
	const std::shared_ptr<ImageExtractData> CorrespondGraph::getImage(const uint32_t& _id) const
	{
		if (imageDatas_.count(_id) == 0)
			return nullptr;
		return imageDatas_.at(_id);
	}
	std::shared_ptr<ImageMatchData> CorrespondGraph::getMatches(const uint32_t& _id1, const uint32_t& _id2)
	{
		uint64_t key;
		site2Key(_id1, _id2, key);
		if (matchDatas_.count(key) == 0)
			return nullptr;
		return matchDatas_[key];
	}
	const std::shared_ptr<ImageMatchData> CorrespondGraph::getMatches(const uint32_t& _id1, const uint32_t& _id2) const
	{
		uint64_t key;
		site2Key(_id1, _id2, key);
		if (matchDatas_.count(key) == 0)
			return nullptr;
		return matchDatas_.at(key);
	}
	std::shared_ptr<CorrespondImage> CorrespondGraph::getCorresponds(const uint32_t& _id)
	{
		if (correspondDatas_.count(_id) == 0)
			return nullptr;
		return correspondDatas_[_id];
	}
	const std::shared_ptr<CorrespondImage> CorrespondGraph::getCorresponds(const uint32_t& _id) const
	{
		if (correspondDatas_.count(_id) == 0)
			return nullptr;
		return correspondDatas_.at(_id);
	}
	CorrespondPoint* CorrespondGraph::getCorrespondPoint(const uint32_t& _imageId, const uint32_t& _pointId, int& _corrNum)
	{
		_corrNum = 0;
		if (correspondDatas_.count(_imageId) == 0)
			return nullptr;
		auto& cit = correspondDatas_[_imageId];
		return cit->getCorrespondPoint(_pointId, _corrNum);
	}
	const CorrespondPoint* CorrespondGraph::getCorrespondPoint(const uint32_t& _imageId, const uint32_t& _pointId, int& _corrNum) const
	{
		_corrNum = 0;
		if (correspondDatas_.count(_imageId) == 0)
			return nullptr;
		auto& cit = correspondDatas_.at(_imageId);
		return cit->getCorrespondPoint(_pointId, _corrNum);
	}
	bool CorrespondGraph::writeData(const std::string& _path, bool bCopyImg)
	{
		if (imageDatas_.empty())
			return false;
		if (!stlplus::folder_exists(_path))
			stlplus::folder_create(_path);
		std::string writeDir = stlplus::create_filespec(_path, "2D");
		if (!stlplus::folder_exists(writeDir))
			stlplus::folder_create(writeDir);
		std::string imageDir = stlplus::create_filespec(writeDir, "images");
		if (bCopyImg && stlplus::folder_exists(imageDir))
			stlplus::folder_delete(imageDir);
		bool cpImg = false;
		if (!stlplus::folder_exists(imageDir)) {
			cpImg = true;
			stlplus::folder_create(imageDir);
		}
		std::string TcwDir = stlplus::create_filespec(writeDir, "Tcws");
		if (stlplus::folder_exists(TcwDir))
			stlplus::folder_delete(TcwDir);
		stlplus::folder_create(TcwDir);
		std::string camPath = stlplus::create_filespec(writeDir, "camera.txt");
		if (stlplus::file_exists(camPath))
			stlplus::file_delete(camPath);
		std::string kpPath = stlplus::create_filespec(writeDir, "KeyPoints.txt");
		if (stlplus::file_exists(kpPath))
			stlplus::file_delete(kpPath);
		std::string mPath = stlplus::create_filespec(writeDir, "Matches.txt");
		if (stlplus::file_exists(mPath))
			stlplus::file_delete(mPath);
		std::string lPath = stlplus::create_filespec(writeDir, "KeyLines.txt");
		if (stlplus::file_exists(lPath))
			stlplus::file_delete(lPath);
		std::string lmPath = stlplus::create_filespec(writeDir, "LineMatches.txt");
		if (stlplus::file_exists(lmPath))
			stlplus::file_delete(lmPath);

		{
			if (imageDatas_.begin()->second->cam)
			{
				SLAM_LYJ::PinholeCamera* camera = dynamic_cast<SLAM_LYJ::PinholeCamera*>(imageDatas_.begin()->second->cam);
				COMMON_LYJ::writePinCamera(camPath, *camera);
			}
		}
		{
			int imgSize = imageDatas_.size();
			std::ofstream kpf(kpPath);
			kpf << "header" << std::endl;
			kpf << "frameSize " << imgSize << std::endl;
			kpf << "data" << std::endl;
			for (auto imgData : imageDatas_)
			{
				if (cpImg)
				{
					std::string imgName = std::to_string(imgData.first) + ".png";
					//std::string imgName = stlplus::filename_part(imgData.second->path);
					std::string imgFile = stlplus::create_filespec(imageDir, imgName);
					stlplus::file_copy(imgData.second->path, imgFile);
				}
				const auto& Tcw = imgData.second->Tcw;
				std::string TcwName = "Tcw" + std::to_string(imgData.first) + ".txt";
				std::string TcwFile = stlplus::create_filespec(TcwDir, TcwName);
				COMMON_LYJ::writeT34(TcwFile, Tcw);
				int kpSize = imgData.second->kps_.size();
				kpf << imgData.first << " " << kpSize << std::endl;
				if (kpSize == 0)
					continue;
				const auto& kps = imgData.second->kps_;
				for (int i = 0; i < kpSize; ++i)
					kpf << kps[i].pt.x << " " << kps[i].pt.y << " ";
				kpf << std::endl;
			}
			kpf.close();
		}
		{
			int framePair = matchDatas_.size();
			std::ofstream mf(mPath);
			mf << "header" << std::endl;
			mf << "framePair " << framePair << std::endl;
			mf << "data" << std::endl;
			for (auto matchData:matchDatas_) {
				uint64_t key = matchData.first;
				uint32_t s1, s2;
				key2Site(key, s1, s2);
				int matchSize = matchData.second->pointMatchSize;
				mf << s1 << " " << s2 << " " << matchSize << std::endl;
				if (matchSize == 0)
					continue;
				const auto& ms = matchData.second->match2to1P;
				int mSize = ms.size();
				for (int i=0;i<mSize;++i)
					if (ms[i] != -1)
						mf << i << " " << ms[i] << " ";
				mf << std::endl;
			}
			mf.close();
		}
		{
			int imgSize = imageDatas_.size();
			std::ofstream lf(lPath);
			lf << "header" << std::endl;
			lf << "frameSize " << imgSize << std::endl;
			lf << "data" << std::endl;
			for (auto imgData : imageDatas_)
			{
				if (!imgData.second->vecLines_.empty()) {
					int lSize = imgData.second->vecLines_.size();
					lf << imgData.first << " " << lSize << std::endl;
					if (lSize == 0)
						continue;
					const auto& ls = imgData.second->vecLines_;
					for (int i = 0; i < lSize; ++i)
						lf << ls[i](0) << " " << ls[i](1) << " " << ls[i](2) << " " << ls[i](3) << " ";
					lf << std::endl;
				}
				else {
					int lSize = imgData.second->vecKeyLines_.size();
					lf << lSize << std::endl;
					if (lSize == 0)
						continue;
					const auto& kls = imgData.second->vecKeyLines_;
					for (int i = 0; i < lSize; ++i)
						lf << kls[i].startPointX << " " << kls[i].startPointY << " " << kls[i].endPointX << " " << kls[i].endPointY << " ";
					lf << std::endl;
				}
			}
			lf.close();
		}
		{
			int framePair = matchDatas_.size();
			std::ofstream lmf(lmPath);
			lmf << "header" << std::endl;
			lmf << "framePair" << framePair << std::endl;
			lmf << "data" << std::endl;
			for (auto matchData : matchDatas_) {
				uint64_t key = matchData.first;
				uint32_t s1, s2;
				key2Site(key, s1, s2);
				int matchSize = matchData.second->lineMatchSize;
				lmf << s1 << " " << s2 << " " << matchSize << std::endl;
				if (matchSize == 0)
					continue;
				const auto& ms = matchData.second->match2to1L;
				int mSize = ms.size();
				for (int i = 0; i < mSize; ++i)
					if (ms[i] != -1)
						lmf << i << " " << ms[i];
				lmf << std::endl;
			}
			lmf.close();
		}

		return true;
	}
	bool CorrespondGraph::readData(const std::string& _path)
	{
		if (!stlplus::folder_exists(_path))
			return false;
		std::string writeDir = stlplus::create_filespec(_path, "2D");
		if (!stlplus::folder_exists(writeDir))
			return false;
		std::string imageDir = stlplus::create_filespec(writeDir, "images");
		if (!stlplus::folder_exists(imageDir))
			return false;
		std::string TcwDir = stlplus::create_filespec(writeDir, "Tcws");
		if (!stlplus::folder_exists(TcwDir))
			return false;
		std::string camPath = stlplus::create_filespec(writeDir, "camera.txt");
		if (!stlplus::file_exists(camPath))
			return false;
		std::string kpPath = stlplus::create_filespec(writeDir, "KeyPoints.txt");
		if (!stlplus::file_exists(kpPath))
			return false;
		std::string mPath = stlplus::create_filespec(writeDir, "Matches.txt");
		if (!stlplus::file_exists(mPath))
			return false;
		std::string lPath = stlplus::create_filespec(writeDir, "KeyLines.txt");
		if (!stlplus::file_exists(lPath))
			return false;
		std::string lmPath = stlplus::create_filespec(writeDir, "LineMatches.txt");
		if (!stlplus::file_exists(lmPath))
			return false;

		{
			COMMON_LYJ::readPinCamera(camPath, cam_);
		}
		{
			std::vector<std::string> imgNames = stlplus::folder_files(imageDir);
			int imgSize = 0;
			std::ifstream kpf(kpPath);
			std::string header = "";
			int imgId = 0;
			int pointSize = 0;
			kpf >> header;
			kpf >> header >> imgSize;
			kpf >> header;
			for (int i = 0; i < imgSize; ++i) {
				kpf >> imgId >> pointSize;
				std::string imgPath = stlplus::create_filespec(imageDir, imgNames[i]);
				//std::string imgIdStr = imgNames[i].substr(0, imgNames[i].find_last_of('.'));
				//int imgId = std::stoi(imgIdStr);
				std::string imgIdStr = std::to_string(imgId);
				std::string TcwName = "Tcw" + imgIdStr + ".txt";
				std::string TcwPath = stlplus::create_filespec(TcwDir, TcwName);
				imageDatas_[imgId].reset(new ImageProcess_LYJ::ImageExtractData());
				auto imgData = imageDatas_[imgId];
				imgData->cam = &cam_;
				imgData->path = imgPath;
				COMMON_LYJ::readT34(TcwPath, imgData->Tcw);
				auto& kps = imgData->kps_;
				kps.resize(pointSize);
				for (int j = 0; j < pointSize; ++j) {
					kpf >> kps[j].pt.x >> kps[j].pt.y;
				}
			}
			kpf.close();
		}
		{
			std::ifstream mf(mPath);
			std::string header = "";
			int framePair = 0, frameId1, frameId2, matchSize = 0, mId1, mId2;
			mf >> header;
			mf >> header >> framePair;
			mf >> header;
			uint64_t pairId = 0;
			for (int i = 0; i < framePair; ++i) {
				mf >> frameId1 >> frameId2 >> matchSize;
				site2Key(frameId1, frameId2, pairId);
				matchDatas_[pairId].reset(new ImageProcess_LYJ::ImageMatchData());
				auto matchData = matchDatas_[pairId];
				matchData->id1 = frameId1;
				matchData->id2 = frameId2;
				matchData->pointMatchSize = matchSize;
				int kp1Size = imageDatas_[frameId1]->kps_.size();
				matchData->match2to1P.assign(kp1Size, -1);
				for (int j = 0; j < matchSize; ++j) {
					mf >> mId1 >> mId2;
					matchData->match2to1P[mId1] = mId2;
				}
			}
			mf.close();
		}

		{
			int imgSize = 0;
			std::ifstream klf(lPath);
			std::string header = "";
			int imgId = 0;
			int lineSize = 0;
			klf >> header;
			klf >> header >> imgSize;
			klf >> header;
			for (int i = 0; i < imgSize; ++i) {
				klf >> imgId >> lineSize;
				auto& ls = imageDatas_[imgId]->vecLines_;
				ls.resize(lineSize);
				for (int j = 0; j < lineSize; ++j) {
					klf >> ls[j][0] >> ls[j][1] >> ls[j][2] >> ls[j][3];
				}
			}
			klf.close();
		}
		{
			std::ifstream lmf(lmPath);
			std::string header = "";
			int framePair = 0, frameId1, frameId2, matchSize = 0, mId1, mId2;
			lmf >> header;
			lmf >> header >> framePair;
			lmf >> header;
			uint64_t pairId = 0;
			for (int i = 0; i < framePair; ++i) {
				lmf >> frameId1 >> frameId2 >> matchSize;
				site2Key(frameId1, frameId2, pairId);
				auto matchData = matchDatas_[pairId];
				matchData->match2to1L.assign(imageDatas_[frameId1]->vecLines_.size(), -1);
				for (int j = 0; j < matchSize; ++j) {
					lmf >> mId1 >> mId2;
					matchData->match2to1L[mId1] = mId2;
				}
			}
			lmf.close();
		}

		generate(imageDatas_, matchDatas_);

		return true;
	}
}