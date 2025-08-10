#include "SIFTextractor.h"


namespace ImageProcess_LYJ{


SIFTExtractor::SIFTExtractor(Option _opt) : ExtractorAbr(ExtractorAbr::TYPE::SIFT), opt_(_opt)
{
    sift_ = cv::xfeatures2d::SiftFeatureDetector::create();
}

SIFTExtractor::~SIFTExtractor()
{
}

void SIFTExtractor::extract(cv::Mat _img, ImageExtractData* _frame)
{
    if (!_frame || !_frame->usePointFeature)
        return;
    sift_->detectAndCompute(_img, cv::Mat(), _frame->kps_, _frame->descriptors_);
}

}