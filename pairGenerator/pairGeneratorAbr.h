#ifndef PAIR_GENERATOR_ABR_H
#define PAIR_GENERATOR_ABR_H


#include "ImageProcess_LYJ_Defines.h"



namespace ImageProcess_LYJ
{
	class PairGeneratorAbr
	{
	public:
		PairGeneratorAbr() {}
		~PairGeneratorAbr() {}

		virtual void generatePairs(std::vector<ImageExtractData>& _extractDatas, std::vector<ImageMatchData>& _matchDatas) = 0;
	protected:

	};



}






#endif
