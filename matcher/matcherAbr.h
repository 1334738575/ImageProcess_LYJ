#ifndef SLAM_LYJ_MATCHERABR_H
#define SLAM_LYJ_MATCHERABR_H

#include "ImageProcess_LYJ_Defines.h"
#include <vector>


namespace ImageProcess_LYJ
{

class MatcherAbr
{
public:
	enum class TYPE
	{
		POINT=0,
		LINE,
		EDGE,
		PATCH,
		OPTICAL
	};
	MatcherAbr(TYPE _type) :type_(_type) {}
	~MatcherAbr() {}

	//_match2to1, size == 1, [ind] == 2
	virtual int match(ImageExtractData* const _frame1, ImageExtractData* const _frame2, ImageMatchData* const _result) = 0;

	TYPE getType() { return type_; }
protected:
	TYPE  type_;
};



}


#endif //SLAM_LYJ_MATCHERABR_H
