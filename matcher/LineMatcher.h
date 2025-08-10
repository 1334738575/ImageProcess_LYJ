#ifndef SLAM_LYJ_LINEMATCHER_H
#define SLAM_LYJ_LINEMATCHER_H

#include "matcherAbr.h"

namespace ImageProcess_LYJ{


class LineMatcher : public MatcherAbr
{
public:
	enum MODE
	{
		LDB=0,
		FUNDAMENTAL
	};
	struct Option
	{
		MODE mode = LDB;
		float ldbTh = 25;
	};
	LineMatcher(Option _opt);
	~LineMatcher();


	// Í¨¹ý MatcherAbr ¼Ì³Ð
	int match(ImageExtractData* const _frame1, ImageExtractData* const _frame2, ImageMatchData* const _result) override;
private:
	Option opt_;
	cv::Ptr<cv::line_descriptor::BinaryDescriptorMatcher> ldb_;
};



}

#endif //SLAM_LYJ_GRIDMATCHER_H