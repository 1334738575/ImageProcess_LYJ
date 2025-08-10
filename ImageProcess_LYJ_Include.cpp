#include "ImageProcess_LYJ_Include.h"
#include "extractor/ORBextractor.h"
#include "matcher/PointMatcher.h"



namespace ImageProcess_LYJ
{


IMAGEPROCESS_LYJ_API void print_ImageProcess_LYJ_Test()
{
    printf("Hello ImageProcess_LYJ!");
}

IMAGEPROCESS_LYJ_API void extractORBFeature(ImageExtractData* _frame)
{
    if (!_frame)
        return;
    ORBExtractor::Option opt;
    ORBExtractor extractor(opt);
    extractor.extract(_frame->img, _frame);
    _frame->featureGrid_ = std::make_shared<FeatureGrid>(_frame->img.cols, _frame->img.rows, 50, _frame->kps_);
    return;
}

IMAGEPROCESS_LYJ_API int matchORBFeature(ImageExtractData* const _frame1, ImageExtractData* const _frame2, ImageMatchData* const _matchResult)
{
    int cnt = 0;
    if (!_matchResult || !_frame1 || !_frame2 || !_matchResult->usePointMatch || !_frame1->usePointFeature || !_frame2->usePointFeature)
        return -1;
    PointMatcher::Option opt;
    opt.mode = 7;
    PointMatcher matcher(opt);
    matcher.match(_frame1, _frame2, _matchResult);
    return cnt;
}



}