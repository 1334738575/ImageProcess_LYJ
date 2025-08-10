#include "EdgeMatcher.h"

namespace ImageProcess_LYJ{


	EdgeMatcher::EdgeMatcher(Option _opt) : MatcherAbr(MatcherAbr::TYPE::EDGE), opt_(_opt)
	{
	}

	EdgeMatcher::~EdgeMatcher()
	{
	}

	int EdgeMatcher::match(ImageExtractData* const _frame1, ImageExtractData* const _frame2, ImageMatchData* const _result)
	{
		if (!_result || !_frame1 || !_frame2 || !_result->useEdgeMatch || !_frame1->useEdgeFeature || !_frame2->useEdgeFeature)
			return -1;
		int fSize1 = (int)_frame1->edges_.size();
		int fSize2 = (int)_frame2->edges_.size();
		_result->match2to1E.assign(fSize1, -1);
		std::vector<std::pair<Eigen::Vector2f, int>> nearPs;
		std::vector<float> dists;
		int nearSize = -1;
		//auto funcDist = [&features1](const SLAM_LYJ_MATH::KdTree<float, 2>::Node* _node, const Eigen::Vector2f& _p)->float {
		//};
		for (int i = 0; i < fSize2; ++i) {
			nearSize = _frame1->kdtree_->search2(_frame2->edges_[i], opt_.nearN, opt_.distTh, nearPs, dists);
			if (nearSize <= 0)
				continue;
			for (int j = 0; j < nearSize; ++j) {
				if(_frame1->edgesDir_[nearPs[j].second].dot(_frame2->edgesDir_[i]) < opt_.dirTh)
					_result->match2to1E[nearPs[i].second] = i;
			}
		}
		return 0;
	}


}