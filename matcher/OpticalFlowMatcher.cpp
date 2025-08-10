#include "OpticalFlowMatcher.h"


namespace ImageProcess_LYJ{

OpticalFlowMatcher::OpticalFlowMatcher(Option _opt)
	:MatcherAbr(TYPE::OPTICAL), m_opt(_opt)
{
}
OpticalFlowMatcher::~OpticalFlowMatcher()
{
}
int OpticalFlowMatcher::match(ImageExtractData* const _frame1, ImageExtractData* const _frame2, ImageMatchData* const _result)
{
	if (!_result || !_frame1 || !_frame2 || !_result->usePointMatch || !_frame1->usePointFeature || !_frame2->usePointFeature)
		return -1;
	matchOpticalFlow(_frame1->img, _frame2->img,
		_frame1->kps_, _frame2->kps_,
		_result->match2to1P, _result->weightsP);
	return 0;
}
int OpticalFlowMatcher::matchOpticalFlow(const cv::Mat& _m1, const cv::Mat& _m2,
	const std::vector<cv::KeyPoint>& _kps1, std::vector<cv::KeyPoint>& _kps2,
	std::vector<int>& _match2to1, std::vector<float>& _weights)
{
	_kps2.clear();
	int pSize = _kps1.size();
	std::vector<cv::Point2f> kps1(pSize);
	for (int i = 0; i < pSize; ++i)
		kps1[i] = _kps1[i].pt;
	std::vector<cv::Point2f> kps2;
	int cnt = matchOpticalFlow(_m1, _m2, kps1, kps2, _match2to1, _weights);
	pSize = kps2.size();
	_kps2.resize(pSize);
	for (int i = 0; i < pSize; ++i)
		_kps2[i].pt = kps2[i];
	return cnt;
}
int OpticalFlowMatcher::matchOpticalFlow(const cv::Mat& _m1, const cv::Mat& _m2,
	const std::vector<cv::Point2f>& _kps1, std::vector<cv::Point2f>& _kps2,
	std::vector<int>& _match2to1, std::vector<float>& _weights)
{
	int pSize = _kps1.size();
	_match2to1.assign(pSize, -1);
	_kps2.clear();
	std::vector<uchar> status;
	std::vector<float> errs;
	cv::calcOpticalFlowPyrLK(_m1, _m2, _kps1, _kps2, status, errs, m_opt.winSize, m_opt.level);
	int cnt = 0;
	for (int i = 0; i < pSize; ++i)
		if (status[i]) {
			_match2to1[i] = i;
			++cnt;
		}
	return cnt;
}


}