#include "LineMatcher.h"

namespace ImageProcess_LYJ{


LineMatcher::LineMatcher(Option _opt) : MatcherAbr(MatcherAbr::TYPE::LINE), opt_(_opt)
{
	if(_opt.mode = LDB)
		ldb_ = cv::line_descriptor::BinaryDescriptorMatcher::createBinaryDescriptorMatcher();
}

LineMatcher::~LineMatcher()
{
}

int LineMatcher::match(ImageExtractData* const _frame1, ImageExtractData* const _frame2, ImageMatchData* const _result)
{
	if (!_result || !_frame1 || !_frame2 || !_result->useLineMatch || !_frame1->useLineFeature || !_frame2->useLineFeature)
		return -1;
	_result->match2to1L.assign(_frame1->vecKeyLines_.size(), -1);
	if (opt_.mode == LDB) {
		std::vector<cv::DMatch> matches;
		ldb_->match(_frame1->lineDescriptors_, _frame2->lineDescriptors_, matches);
		for (size_t i = 0; i < matches.size(); ++i) {
			if (matches[i].distance < opt_.ldbTh) {
				_result->match2to1L[matches[i].queryIdx] = matches[i].trainIdx;
			}
		}
	}
	else if (opt_.mode == FUNDAMENTAL) {
		Eigen::Matrix3d F = SLAM_LYJ::SLAM_LYJ_MATH::calculateFundamentalMatrix(
			_frame1->Tcw.getR(), _frame1->Tcw.gett(), _frame1->cam->getK(),
			_frame2->Tcw.getR(), _frame2->Tcw.gett(), _frame2->cam->getK()
		);
	}
	return 0;
}



}