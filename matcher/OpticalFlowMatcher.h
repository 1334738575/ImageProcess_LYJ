#ifndef SLAM_LYJ_OPTICALFLOW_H
#define SLAM_LYJ_OPTICALFLOW_H

#include "matcherAbr.h"

namespace ImageProcess_LYJ{


class OpticalFlowMatcher : public MatcherAbr
{
public:
	struct Option
	{
		cv::Size winSize = cv::Size(21, 21);
		int level = 3;
	};

	OpticalFlowMatcher(Option _opt);
	~OpticalFlowMatcher();



	// Í¨¹ý MatcherAbr ¼Ì³Ð
	int match(ImageExtractData* const _frame1, ImageExtractData* const _frame2, ImageMatchData* const _result) override;
	int matchOpticalFlow(const cv::Mat& _m1, const cv::Mat& _m2,
		const std::vector<cv::KeyPoint>& _kps1, std::vector<cv::KeyPoint>& _kps2,
		std::vector<int>& _match2to1, std::vector<float>& _weights);
	int matchOpticalFlow(const cv::Mat& _m1, const cv::Mat& _m2,
		const std::vector<cv::Point2f>& _kps1, std::vector<cv::Point2f>& _kps2,
		std::vector<int>& _match2to1, std::vector<float>& _weights);

private:
	Option m_opt;
};






}

#endif // !SLAM_LYJ_OPTICALFLOW_H
