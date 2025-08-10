#ifndef SLAM_LYJ_SIFTEXTRACTOR_H
#define SLAM_LYJ_SIFTEXTRACTOR_H

#include "extractorAbr.h"
#include <opencv2/xfeatures2d.hpp>

namespace ImageProcess_LYJ{

class SIFTExtractor : public ExtractorAbr
{
public:
	struct Option
	{

	};
	SIFTExtractor(Option _opt);
	~SIFTExtractor();

	// Í¨¹ý ExtractorAbr ¼Ì³Ð
	void extract(cv::Mat _img, ImageExtractData* _frame) override;

private:
	cv::Ptr<cv::xfeatures2d::SiftFeatureDetector> sift_;
	Option opt_;
};


}

#endif //SLAM_LYJ_SIFTEXTRACTOR_H