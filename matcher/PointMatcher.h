#ifndef SLAM_LYJ_POINTMATCHER_H
#define SLAM_LYJ_POINTMATCHER_H

#include "matcherAbr.h"

namespace ImageProcess_LYJ{

class PointMatcher : public MatcherAbr
{
public:
	/*
	*	NULL				  = 0,
	    FLANNBASED            = 1,
        BRUTEFORCE            = 2,
        BRUTEFORCE_L1         = 3,
        BRUTEFORCE_HAMMING    = 4,
        BRUTEFORCE_HAMMINGLUT = 5,
        BRUTEFORCE_SL2        = 6,
		USER_DEFINE			  >=7
	*/
	struct Option
	{
		int mode = 1;
		//float distTh = 25;
		float squareDThInMesh = 0.01;
	};
	PointMatcher(Option _opt);
	~PointMatcher();

	// Í¨¹ý MatcherAbr ¼Ì³Ð
	int match(ImageExtractData* const _frame1, ImageExtractData* const _frame2, ImageMatchData* const _result) override;
private:
	Option opt_;
	cv::Ptr<cv::DescriptorMatcher> descMatcher_;
	std::shared_ptr<cv::BFMatcher> bfMatcher_;
};



}

#endif //SLAM_LYJ_BFMATCHER_H