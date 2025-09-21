#include "Cannyextractor.h"


namespace ImageProcess_LYJ{


CannyExtractor::CannyExtractor(Option _opt) :ExtractorAbr(ExtractorAbr::TYPE::CANNY), opt_(_opt)
{
}

CannyExtractor::~CannyExtractor()
{
}

void CannyExtractor::extract(cv::Mat _img, ImageExtractData* _frame)
{
	if (!_frame || !_frame->useEdgeFeature)
		return;
	const int& downSample = opt_.step;
	if (opt_.mode == 1 && _frame->useLineFeature)
	{
		int lenTh = 5 * downSample;
		cv::Mat edgeM(_frame->img.rows, _frame->img.cols, CV_16SC1);
		edgeM.setTo(INT16_MAX);
		const auto& lines = _frame->vecLines_;
		const auto& lines2f = _frame->lines2f;
		for (int i = 0; i < lines.size(); ++i)
		{
			if (lines2f[i].length < lenTh)
				continue;
			cv::line(edgeM, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), i);
		}
		Eigen::Vector2f uvTmp;
		for (int i = 0; i < _img.rows; i += downSample) {
			for (int j = 0; j < _img.cols; j += downSample) {
				const auto& v = edgeM.at<int16_t>(i, j);
				if (v == INT16_MAX)
					continue;
				uvTmp(0) = static_cast<float>(j);
				uvTmp(1) = static_cast<float>(i);
				_frame->edges_.push_back(uvTmp);
				_frame->edgesDir_.push_back(lines2f[v].dir());
			}
		}
	}
	else
	{
		//dx dy canny
		cv::Sobel(_img, _frame->dxM, CV_16SC1, 1, 0);
		cv::Sobel(_img, _frame->dyM, CV_16SC1, 0, 1);
		cv::Canny(_frame->dxM, _frame->dyM, _frame->cannyM, opt_.lowTh, opt_.highTh);
		//uvdir
		int preSize = _img.rows * _img.cols / 4;
		_frame->edgesDir_.reserve(preSize);
		_frame->edges_.reserve(preSize);
		_frame->isParallel_ = false;
		_frame->nParallelVar_.setZero();
		_frame->nParallel_.setZero();
		Eigen::Vector2f dirTmp;
		Eigen::Vector2f uvTmp;
		int cnt = 0;
		for (int i = 0; i < _img.rows; i += downSample) {
			for (int j = 0; j < _img.cols; j += downSample) {
				auto v = ((uchar*)_frame->cannyM.data)[i * _img.cols + j];
				if (v <= 0)
					continue;
				uvTmp(0) = static_cast<float>(j);
				uvTmp(1) = static_cast<float>(i);
				dirTmp(0) = -1 * static_cast<float>(((int16_t*)_frame->dyM.data)[i * _img.cols + j]);
				dirTmp(1) = static_cast<float>(((int16_t*)_frame->dxM.data)[i * _img.cols + j]);
				dirTmp.normalize();
				_frame->edges_.push_back(uvTmp);
				_frame->edgesDir_.push_back(dirTmp);
			}
		}
	}
	if (_frame->depths.empty())
		return;
	auto& edges = _frame->edges_;
	auto& edge3Ds = _frame->edge3Ds_;
	auto& edge3DNors = _frame->edge3DNors_;
	edge3Ds.assign(edges.size(), Eigen::Vector3f(0,0,0));
	edge3DNors.assign(edges.size(), Eigen::Vector3f(0,0,0));
	int r, c;
	for (int i = 0; i < edges.size(); ++i)
	{
		r = edges[i](1);
		c = edges[i](0);
		const float& d = _frame->depths.at<float>(r, c);
		if (d == FLT_MAX)
			continue;
		_frame->cam->image2World(edges[i](0), edges[i](1), d, edge3Ds[i]);
		if (!_frame->fIds.empty() && _frame->btmPtr && _frame->btmPtr->isEnableFNormals()) {
			const uint32_t fId = _frame->fIds.at(r * _frame->cam->wide() + c);
			if (fId == UINT32_MAX)
				continue;
			edge3DNors[i] = _frame->btmPtr->getFNormal(fId);
		}
	}
}

}