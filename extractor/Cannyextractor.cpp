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
	cv::Sobel(_img, _frame->dxM, CV_16SC1, 1, 0);
	cv::Sobel(_img, _frame->dxM, CV_16SC1, 0, 1);
	cv::Canny(_frame->dxM, _frame->dyM, _frame->cannyM, opt_.lowTh, opt_.highTh);
	return; //后续步骤建议结合3D信息的情况下做
	int preSize = _img.rows * _img.cols / 4;
	_frame->edgesDir_.reserve(preSize);
	_frame->edges_.reserve(preSize);
	_frame->isParallel_ = false;
	_frame->nParallelVar_.setZero();
	_frame->nParallel_.setZero();
	Eigen::Vector2f dirTmp;
	Eigen::Vector2f uvTmp;
	int cnt = 0;
	for (int i = 0; i < _img.rows; ++i) {
		for (int j = 0; j < _img.cols; ++j) {
			auto v = ((int16_t*)_frame->cannyM.data)[i * _img.cols + j];
			if (v <= 0)
				continue;
			uvTmp(0) = static_cast<float>(j);
			uvTmp(1) = static_cast<float>(i);
			dirTmp(0) = static_cast<float>(((int16_t*)_frame->dxM.data)[i * _img.cols + j]);
			dirTmp(1) = static_cast<float>(((int16_t*)_frame->dyM.data)[i * _img.cols + j]);
			_frame->edges_.push_back(uvTmp);
			_frame->edgesDir_.push_back(dirTmp);
		}
	}
}

}