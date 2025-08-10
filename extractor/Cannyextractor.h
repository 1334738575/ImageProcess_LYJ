#ifndef IMAGEPROCESS_LYJ_CANNYEXTRACTOR_H
#define IMAGEPROCESS_LYJ_CANNYEXTRACTOR_H

#include "extractorAbr.h"

namespace ImageProcess_LYJ{

class CannyExtractor : public ExtractorAbr
{
public:
	struct Option
	{
		int lowTh = 50;
		int highTh = 140;
	};
	CannyExtractor(Option _opt);
	~CannyExtractor();

	// Í¨¹ý ExtractorAbr ¼Ì³Ð
	void extract(cv::Mat _img, ImageExtractData* _frame) override;

private:
	Option opt_;
};



}

#endif //IMAGEPROCESS_LYJ_CANNYEXTRACTOR_H