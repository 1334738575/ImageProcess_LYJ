#include "bfPairGenerator.h"



namespace ImageProcess_LYJ
{

	BFPairGenerator::BFPairGenerator(Option _opt)
		:opt_(_opt)
	{}
	BFPairGenerator::~BFPairGenerator()
	{}

	void BFPairGenerator::generatePairs(std::vector<ImageExtractData>& _extractDatas, std::vector<ImageMatchData>& _matchDatas)
	{
		int imgSz = _extractDatas.size();
		int matchDataSz = imgSz * (imgSz - 1) / 2;
		_matchDatas.resize(matchDataSz);
		int cnt = 0;
		for (int i = 0; i < imgSz-1; ++i)
		{
			for (int j = i+1; j < imgSz; ++j)
			{
				_matchDatas[cnt].id1 = i;
				_matchDatas[cnt].id2 = j;
			}
		}
	}


}