#include "ImageProcess_LYJ_Include.h"
#include "extractor/ORBextractor.h"
#include "matcher/PointMatcher.h"
#include "ImageCommon/TwoViewReconstruction.h"
#include <base/Triangler.h>

namespace ImageProcess_LYJ
{

    IMAGEPROCESS_LYJ_API void print_ImageProcess_LYJ_Test()
    {
        printf("Hello ImageProcess_LYJ!");
    }

    IMAGEPROCESS_LYJ_API void extractFeature(ImageExtractData *_frame, const ImageExtractOption& _opt)
    {
        if (!_frame)
            return;
        if (_opt.usePointFeature && _opt.pointExtractMode == 0) {
            ORBExtractor::Option opt;
            ORBExtractor extractor(opt);
            extractor.extract(_frame->img, _frame);
            _frame->featureGrid_ = std::make_shared<FeatureGrid>(_frame->img.cols, _frame->img.rows, 50, _frame->kps_);
        }
        return;
    }

    IMAGEPROCESS_LYJ_API int matchFeature(ImageExtractData *const _frame1, ImageExtractData *const _frame2, ImageMatchData *const _matchResult, const ImageMatchOption& _opt)
    {
        int cnt = 0;
        if (!_matchResult || !_frame1 || !_frame2 || !_matchResult->usePointMatch || !_frame1->usePointFeature || !_frame2->usePointFeature)
            return -1;
        PointMatcher::Option opt;
        opt.mode = _opt.pointMatchMode;
        PointMatcher matcher(opt);
        cnt = matcher.match(_frame1, _frame2, _matchResult);
        return cnt;
    }

    IMAGEPROCESS_LYJ_API bool reconstructTwo(ImageExtractData *const _frame1, ImageExtractData *const _frame2, ImageMatchData *const _matchResult, ImageTriangleData *const _triangleResult, const ImageTriangleOption& _opt)
    {
        if (!_frame1 || !_frame2 || !_matchResult || !_triangleResult)
            return false;
        if (_opt.justTri)
        {
            SLAM_LYJ::TriangleOption opt;
            SLAM_LYJ::Triangler<double> triangler(opt);
            int sz = _frame1->kps_.size();
            _triangleResult->bTris.assign(sz, false);
            _triangleResult->Ps.resize(sz, Eigen::Vector3d::Zero());
            SLAM_LYJ::PinholeCamera *cam = dynamic_cast<SLAM_LYJ::PinholeCamera *>(_frame1->cam);
            std::vector<Eigen::Vector2d> uvs(2);
            std::vector<SLAM_LYJ::Pose3D> Tcws(2);
            Tcws[0] = _frame1->Tcw;
            Tcws[1] = _frame2->Tcw;
            std::vector<SLAM_LYJ::PinholeCamera> cams(2);
            cams[0] = *cam;
            cams[1] = *cam;
            for (int i = 0; i < sz; ++i)
            {
                if (_matchResult->match2to1P[i] == -1)
                    continue;
                uvs[0](0) = _frame1->kps_[i].pt.x;
                uvs[0](1) = _frame1->kps_[i].pt.y;
                uvs[1](0) = _frame2->kps_[_matchResult->match2to1P[i]].pt.x;
                uvs[1](1) = _frame2->kps_[_matchResult->match2to1P[i]].pt.y;
                if (triangler.runDirect(uvs, Tcws, cams, _triangleResult->Ps[i]))
                    _triangleResult->bTris[i] = true;
            }
        }
        else
        {
            TwoViewReconstruction reconstruction(_frame1->cam->getK().cast<float>());
            std::vector<cv::Point3f> P1s;
            reconstruction.Reconstruct(_frame1->kps_, _frame2->kps_, _matchResult->match2to1P, _triangleResult->T21, P1s, _triangleResult->bTris);
            _triangleResult->Ps.resize(P1s.size());
            for (size_t i = 0; i < P1s.size(); ++i)
            {
                _triangleResult->Ps[i] = Eigen::Vector3f(P1s[i].x, P1s[i].y, P1s[i].z).cast<double>();
            }
        }
        return true;
    }

}