#ifndef LOC_PAIR_GENERATOR_H
#define LOC_PAIR_GENERATOR_H

#include "pairGeneratorAbr.h"


namespace ImageProcess_LYJ
{
	class LocPairGenerator : public PairGeneratorAbr
	{
	public:
		struct Option
		{
			int nn = 10;
			float distTh = 1;
		};
		LocPairGenerator(Option _opt);
		~LocPairGenerator();


		// Í¨¹ý PairGeneratorAbr ¼Ì³Ð
		void generatePairs(std::vector<ImageExtractData>& _extractDatas, std::vector<ImageMatchData>& _matchDatas) override;


	private:
		Option opt_;

	};


}


#endif // !LOC_PAIR_GENERATOR_H
