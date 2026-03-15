#ifndef VOC_PAIR_GENERATOR_H
#define VOC_PAIR_GENERATOR_H

#include "pairGeneratorAbr.h"
#include <DBoW3/Vocabulary.h>
#include <DBoW3/Database.h>

namespace ImageProcess_LYJ
{
	class VocPairGenerator : public PairGeneratorAbr
	{
	public:
		struct Option
		{
			int nn = 10;
			double scoreTh = 0.5;
			std::string vocPath = "";
		};
		VocPairGenerator(Option _opt);
		~VocPairGenerator();


		// 通过 PairGeneratorAbr 继承
		void generatePairs(std::vector<ImageExtractData>& _extractDatas, std::vector<ImageMatchData>& _matchDatas) override;

		virtual bool loadVoc() = 0;
		virtual void transformFrames(std::vector<ImageExtractData>& _frames) = 0;
		virtual void queryFrame(ImageExtractData& _frame, std::vector<int>& _rets) = 0;

	protected:
		Option opt_;

	};


	class ORBVocPairGenerator : public VocPairGenerator
	{
	public:
		ORBVocPairGenerator(Option _opt);
		~ORBVocPairGenerator();

		// 通过 VocPairGenerator 继承
		bool loadVoc() override;

		void transformFrames(std::vector<ImageExtractData>& _frames) override;

		void queryFrame(ImageExtractData& _frame, std::vector<int>& _rets) override;

	private:

		std::shared_ptr<DBoW3::Vocabulary> orbVoc_ = nullptr;
		std::shared_ptr<DBoW3::Database> orbDB_ = nullptr;
	};

}


#endif // !VOC_PAIR_GENERATOR_H
