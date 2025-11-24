#include "LSDextractor.h"

namespace ImageProcess_LYJ{


LSDExtractor::LSDExtractor(Option _opt) : ExtractorAbr(ExtractorAbr::TYPE::LSD), opt_(_opt)
{
    if(!_opt.needDesc)
        lsd_ = cv::createLineSegmentDetector();
    else {
        lsdKeyLine_ = cv::line_descriptor::LSDDetector::createLSDDetector();
        lineDescriptor_ = cv::line_descriptor::BinaryDescriptor::createBinaryDescriptor();
    }
}

LSDExtractor::~LSDExtractor()
{
}

void LSDExtractor::extract(cv::Mat _img, ImageExtractData* _frame)
{
    if (!_frame || !_frame->useLineFeature)
        return;
    if (!opt_.needDesc) {
        lsd_->detect(_img, _frame->vecLines_);
        _frame->lines2f.resize(_frame->vecLines_.size());
		for (int i = 0; i < _frame->vecLines_.size(); ++i) {
			_frame->lines2f[i] = SLAM_LYJ::Line2f(Eigen::Vector2f(_frame->vecLines_[i][0], _frame->vecLines_[i][1]), Eigen::Vector2f(_frame->vecLines_[i][2], _frame->vecLines_[i][3]));
		}
        if (_frame->depths.empty())
            return;
        _frame->line3Ds_.assign(_frame->vecLines_.size(), SLAM_LYJ::Line3f());
        for (int i = 0; i < _frame->vecLines_.size(); ++i) {
            getLine3D(_frame->vecLines_[i][0], _frame->vecLines_[i][1], _frame->vecLines_[i][2], _frame->vecLines_[i][3], _frame->depths, _frame->cam, _frame->line3Ds_[i]);
        }
    }
    else {
        lsdKeyLine_->detect(_img, _frame->vecKeyLines_, 2, 2);
        lineDescriptor_->compute(_img, _frame->vecKeyLines_, _frame->lineDescriptors_);
        if (_frame->depths.empty())
            return;
        _frame->line3Ds_.assign(_frame->vecKeyLines_.size(), SLAM_LYJ::Line3f());
        for (int i = 0; i < _frame->vecKeyLines_.size(); ++i) {
            getLine3D(_frame->vecKeyLines_[i].startPointX, _frame->vecKeyLines_[i].startPointY, _frame->vecKeyLines_[i].endPointX, _frame->vecKeyLines_[i].endPointY, _frame->depths, _frame->cam, _frame->line3Ds_[i]);
        }
    }
}

void LSDExtractor::convertKeyLine2CVVec4f(const std::vector<cv::line_descriptor::KeyLine>& _keyLines, std::vector<cv::Vec4f>& _lines)
{
    _lines.resize(_keyLines.size());
    for (size_t i = 0; i < _keyLines.size(); ++i) {
        _lines[i](0) = _keyLines[i].startPointX;
        _lines[i](1) = _keyLines[i].startPointY;
        _lines[i](2) = _keyLines[i].endPointX;
        _lines[i](3) = _keyLines[i].endPointY;
    }
}

void LSDExtractor::convertCVVec4f2KeyLine(const std::vector<cv::Vec4f>& _lines, std::vector<cv::line_descriptor::KeyLine>& _keyLines)
{
    _keyLines.resize(_lines.size());
    float dx = 0;
    float dy = 0;
    for (size_t i = 0; i < _lines.size(); ++i) {
        _keyLines[i].startPointX = _lines[i](0);
        _keyLines[i].startPointY = _lines[i](1);
        _keyLines[i].endPointX = _lines[i](2);
        _keyLines[i].endPointY = _lines[i](3);
        _keyLines[i].class_id = static_cast<int>(i);
        dx = _lines[i](2) - _lines[i](0);
        dy = _lines[i](3) - _lines[i](1);
        _keyLines[i].angle = atan2(dy, dx);
    }
}

bool LSDExtractor::getLine3D(const float& _sx, const float& _sy, const float& _ex, const float& _ey, const cv::Mat& _depths, SLAM_LYJ::CameraModule* _cam, SLAM_LYJ::Line3f& _line3D)
{
    return false;

    SLAM_LYJ::SLAM_LYJ_MATH::RANSAC<Eigen::Vector3f, float, SLAM_LYJ::Line3f> ransac(0.8, 2);
    auto funcCalErr = [](const SLAM_LYJ::Line3f& _L3D, const Eigen::Vector3f& _P, float& _err)->bool
        {
            _err = _L3D.distP2L(_P);
            if (_err > 0.01)
                return false;
            return true;
        };
    auto funcCalMod = [](const std::vector<const Eigen::Vector3f*>& _samples, SLAM_LYJ::Line3f& _mdl)->bool
        {
            _mdl.update(*(_samples[0]), *(_samples[1]));
            return _mdl.length > 0;
        };
    ransac.setCallBack(funcCalErr, funcCalMod);

    std::vector<Eigen::Vector2i> pixs;
    SLAM_LYJ::Line2d::bresenhamLine(_sx, _sy, _ex, _ey, pixs);
    std::vector<Eigen::Vector3f> P3Ds;
    P3Ds.reserve(pixs.size());
    Eigen::Vector3f PTmp;
    for (int i = 0; i < pixs.size(); ++i)
    {
        const float& d = _depths.at<float>(pixs[i](1), pixs[i](0));
        if (d == FLT_MAX)
            continue;
        _cam->image2World(pixs[i](0), pixs[i](1), d, PTmp);
        P3Ds.push_back(PTmp);
    }
    std::vector<float> errs;
    std::vector<bool> inliners;
    double inRatio = ransac.run(P3Ds, errs, inliners, _line3D);
    return inRatio > 0.8;
}


}