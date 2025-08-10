#ifndef SLAM_LYJ_EDGEMATCHER_H
#define SLAM_LYJ_EDGEMATCHER_H

#include "matcherAbr.h"

namespace ImageProcess_LYJ{

class EdgeMatcher : public MatcherAbr
{
public:
	struct Option
	{
		float dirTh = 0.9f; //around -20degree, +20degree
		int nearN = 3;
		float distTh = 10;
	};
	EdgeMatcher(Option _opt);
	~EdgeMatcher();



	// Í¨¹ý MatcherAbr ¼Ì³Ð
	int match(ImageExtractData* const _frame1, ImageExtractData* const _frame2, ImageMatchData* const _result) override;

private:
	Option opt_;
};




}

#endif //SLAM_LYJ_KDTREEMATCHER_H