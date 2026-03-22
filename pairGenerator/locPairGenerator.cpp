#include "locPairGenerator.h"
#include <common/FlannSearch.h> 


namespace ImageProcess_LYJ
{

	LocPairGenerator::LocPairGenerator(Option _opt)
		:opt_(_opt)
	{}
	LocPairGenerator::~LocPairGenerator()
	{}

	void LocPairGenerator::generatePairs(std::vector<ImageExtractData>& _extractDatas, std::vector<ImageMatchData>& _matchDatas)
	{
		int imgSz = _extractDatas.size();
		std::vector<Eigen::Vector3f> locs(imgSz);
		for (int i = 0; i < imgSz; ++i)
		{
			COMMON_LYJ::Pose3D Twc = _extractDatas[i].Tcw.inversed();
			locs[i] = Twc.gett().cast<float>();
		}
		size_t nn = (imgSz - 1) > (opt_.nn+1) ? (opt_.nn+1) : (imgSz - 1);
		float distTh = opt_.distTh;
		int matchDataSz = imgSz * nn;
		_matchDatas.reserve(matchDataSz);

		COMMON_LYJ::FLANNWrapper<float, 3> flann;
		flann.build_index(locs[0].data(), imgSz);
		std::vector<int> inds;
		std::vector<float> dists;
		ImageMatchData matchData;
		for (int i = 0; i < imgSz; ++i)
		{
			flann.batch_search<1>(locs[i].data(), nn, inds, dists);
			if (inds.empty())
				continue;
			for (int j = 0; j < inds.size(); ++j)
			{
				if (inds[j] <= i)
					continue;
				if (dists[j] > distTh)
					continue;
				matchData.id1 = i;
				matchData.id2 = j;
				_matchDatas.push_back(matchData);
			}
		}
	}


}