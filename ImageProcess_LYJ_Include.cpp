#include "ImageProcess_LYJ_Include.h"

#include "extractor/ORBextractor.h"
#include "extractor/Cannyextractor.h"
#include "extractor/LSDextractor.h"

#include "matcher/PointMatcher.h"

#include "ImageCommon/TwoViewReconstruction.h"

#include <base/Triangler.h>
#include <DBow3/Vocabulary.h>

namespace ImageProcess_LYJ
{

    IMAGEPROCESS_LYJ_API void print_ImageProcess_LYJ_Test()
    {
        printf("Hello ImageProcess_LYJ!");
    }

    IMAGEPROCESS_LYJ_API void testDBoW3(std::vector<cv::Mat>& features)
    {
        using namespace DBoW3;
        using namespace std;
        // branching factor and depth levels
        const int k = 9;
        const int L = 3;
        const DBoW3::WeightingType weight = TF_IDF;
        const ScoringType score = L1_NORM;

        DBoW3::Vocabulary voc(k, L, weight, score);

        cout << "Creating a small " << k << "^" << L << " vocabulary..." << endl;
        voc.create(features);
        cout << "... done!" << endl;
        cout << "Vocabulary information: " << endl
            << voc << endl << endl;

        // lets do something with this vocabulary
        cout << "Matching images against themselves (0 low, 1 high): " << endl;
        BowVector v1, v2;
        for (size_t i = 0; i < features.size(); i++)
        {
            voc.transform(features[i], v1);
            for (size_t j = 0; j < features.size(); j++)
            {
                voc.transform(features[j], v2);
                double score = voc.score(v1, v2);
                cout << "Image " << i << " vs Image " << j << ": " << score << endl;
            }
        }

        // save the vocabulary to disk
        cout << endl << "Saving vocabulary..." << endl;
        voc.save("D:/tmp/small_voc.yml.gz");
        cout << "Done" << endl;
        return;
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
            _frame->featureGridFromORB_ = std::make_shared<FeatureGridFromORB>(_frame->cam->wide(), _frame->cam->height(), &_frame->kps_);
        }
        if (_opt.useLineFeature)
        {
            LSDExtractor::Option opt;
            LSDExtractor extractor(opt);
            extractor.extract(_frame->img, _frame);
        }
        if (_opt.useEdgeFeature)
        {
            CannyExtractor::Option opt;
            CannyExtractor extractor(opt);
            extractor.extract(_frame->img, _frame);
        }
        return;
    }

    IMAGEPROCESS_LYJ_API int matchFeature(ImageExtractData *const _frame1, ImageExtractData *const _frame2, ImageMatchData *const _matchResult, const ImageMatchOption& _opt)
    {
        int cnt = 0;
        if (!_matchResult || !_frame1 || !_frame2 || !_matchResult->usePointMatch || !_frame1->usePointFeature || !_frame2->usePointFeature)
            return -1;
        if (_matchResult->usePointMatch) {
            PointMatcher::Option opt;
            opt.mode = _opt.pointMatchMode;
            PointMatcher matcher(opt);
            cnt = matcher.match(_frame1, _frame2, _matchResult);
            if (_opt.bTriangle) {
                ImageTriangleOption triOpt;
                triOpt.justTri = false;
                if(cnt > 8)
                    reconstructTwo(_frame1, _frame2, _matchResult, &_matchResult->triDatas, triOpt);
            }
        }
        if (_matchResult->useEdgeMatch)
        {

        }
        return cnt;
    }

    IMAGEPROCESS_LYJ_API bool reconstructTwo(ImageExtractData *const _frame1, ImageExtractData *const _frame2, ImageMatchData *const _matchResult, ImageTriangleData *const _triangleResult, const ImageTriangleOption& _opt)
    {
        if (!_frame1 || !_frame2 || !_matchResult || !_triangleResult)
            return false;
        bool ret = true;
        int cnt = 0;
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
                if (triangler.runDirect(uvs, Tcws, cams, _triangleResult->Ps[i])) {
                    _triangleResult->bTris[i] = true;
                    ++cnt;
                }
            }
        }
        else
        {
            TwoViewReconstruction reconstruction(_frame1->cam->getK().cast<float>());
            std::vector<cv::Point3f> P1s;
            ret = reconstruction.Reconstruct(_frame1->kps_, _frame2->kps_, _matchResult->match2to1P, _triangleResult->T21, P1s, _triangleResult->bTris);
            _triangleResult->Ps.resize(P1s.size());
            for (size_t i = 0; i < P1s.size(); ++i)
            {
                if (_triangleResult->bTris[i])
                    ++cnt;
                _triangleResult->Ps[i] = Eigen::Vector3f(P1s[i].x, P1s[i].y, P1s[i].z).cast<double>();
            }
        }
        _triangleResult->triSize = cnt;
        _triangleResult->triSuccess = ret;
        return ret;
    }

}