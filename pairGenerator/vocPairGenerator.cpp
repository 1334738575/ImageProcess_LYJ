#include "vocPairGenerator.h"
#include <STLPlus/include/file_system.h>


namespace ImageProcess_LYJ
{

	VocPairGenerator::VocPairGenerator(Option _opt)
		:opt_(_opt)
	{}
	VocPairGenerator::~VocPairGenerator()
	{}

	void VocPairGenerator::generatePairs(std::vector<ImageExtractData>& _extractDatas, std::vector<ImageMatchData>& _matchDatas)
	{
		int imgSz = _extractDatas.size();
		_matchDatas.clear();
		if (!loadVoc())
			return;
		transformFrames(_extractDatas);
		int nn = opt_.nn + 1;
		int matchDataSz = imgSz * nn;
		_matchDatas.reserve(matchDataSz);
		std::vector<int> simInds;
		ImageMatchData matchData;
		for (int i = 0; i < imgSz; ++i)
		{
			queryFrame(_extractDatas[i], simInds);
			if (simInds.empty())
				continue;
			for (int j = 0; j < simInds.size(); ++j)
			{
				if (simInds[j] <= i)
					continue;
				matchData.id1 = i;
				matchData.id2 = j;
				_matchDatas.push_back(matchData);
			}
		}
	}



	ORBVocPairGenerator::ORBVocPairGenerator(Option _opt)
		:VocPairGenerator(_opt)
	{}
	ORBVocPairGenerator::~ORBVocPairGenerator()
	{}

	bool ORBVocPairGenerator::loadVoc()
	{
		if (!stlplus::file_exists(opt_.vocPath))
			return false;
		orbVoc_.reset(new DBoW3::Vocabulary(opt_.vocPath));
		return true;
	}
	void ORBVocPairGenerator::transformFrames(std::vector<ImageExtractData>& _frames)
	{
		orbDB_.reset(new DBoW3::Database(*orbVoc_, false, 0));
		int imgSz = _frames.size();
		for (int i = 0; i < imgSz; ++i)
		{
			orbVoc_->transform(_frames[i].descriptors_, _frames[i].bowVec, _frames[i].featureVec, 4);
			orbDB_->add(_frames[i].bowVec, _frames[i].featureVec);
		}
	}
	void ORBVocPairGenerator::queryFrame(ImageExtractData& _frame, std::vector<int>& _rets)
	{
		_rets.clear();
		// 检索相似帧
		DBoW3::QueryResults results;
		orbDB_->query(_frame.bowVec, results, opt_.nn);
		if (results.empty())
			return;

		// 整理结果
		for (const auto& res : results) {
			if (res.Score < opt_.scoreTh)
				continue;
			_rets.push_back(res.Id);
		}
		return;
	}

}