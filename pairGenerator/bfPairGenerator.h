#ifndef BF_PAIR_GENERATOR_H
#define BF_PAIR_GENERATOR_H

#include "pairGeneratorAbr.h"


namespace ImageProcess_LYJ
{
	class BFPairGenerator : public PairGeneratorAbr
	{
	public:
		struct Option
		{

		};
		BFPairGenerator(Option _opt);
		~BFPairGenerator();


		// Í¨¹ý PairGeneratorAbr ¼Ì³Ð
		void generatePairs(std::vector<ImageExtractData>& _extractDatas, std::vector<ImageMatchData>& _matchDatas) override;
	

	private:
		Option opt_;
	
	};


}


#endif // !BF_PAIR_GENERATOR_H
