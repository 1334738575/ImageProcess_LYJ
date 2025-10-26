#include "PointMatcher.h"
#include <DBoW3/Vocabulary.h>
#include "ImageCommon/FundamentalEstimator.h"
#include "ImageCommon/HomographyEstimator.h"

namespace ImageProcess_LYJ
{
    const static int TH_HIGH = 100;
    const static int TH_LOW = 50;
    const static int HISTO_LENGTH = 30;

    // static int imgi = 0;
    static void ComputeThreeMaxima(std::vector<int>* histo, const int L, int& ind1, int& ind2, int& ind3)
    {
        int max1 = 0;
        int max2 = 0;
        int max3 = 0;

        for (int i = 0; i < L; i++)
        {
            const int s = histo[i].size();
            if (s > max1)
            {
                max3 = max2;
                max2 = max1;
                max1 = s;
                ind3 = ind2;
                ind2 = ind1;
                ind1 = i;
            }
            else if (s > max2)
            {
                max3 = max2;
                max2 = s;
                ind3 = ind2;
                ind2 = i;
            }
            else if (s > max3)
            {
                max3 = s;
                ind3 = i;
            }
        }

        if (max2 < 0.1f * (float)max1)
        {
            ind2 = -1;
            ind3 = -1;
        }
        else if (max3 < 0.1f * (float)max1)
        {
            ind3 = -1;
        }
    }
    // Bit set count operation from
    // http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
    static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
    {
        const int *pa = a.ptr<int32_t>();
        const int *pb = b.ptr<int32_t>();

        int dist = 0;

        for (int i = 0; i < 8; i++, pa++, pb++)
        {
            unsigned int v = *pa ^ *pb;
            v = v - ((v >> 1) & 0x55555555);
            v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
            dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
        }

        return dist;
    }
    static int matchByF(
        const Eigen::Matrix3d &K1,
        const std::vector<cv::KeyPoint> &features1, const std::vector<cv::KeyPoint> &features2,
        const cv::Mat &desc1, const cv::Mat &desc2,
        FeatureGrid *grid1,
        const Eigen::Matrix3d &rota, const Eigen::Vector3d &trans,
        std::vector<int> &results,
        // std::vector<std::pair<int, int>>& results,
        double th = 50, double nnTh = 0.6,
        cv::Mat img1 = cv::Mat(), cv::Mat img2 = cv::Mat(),
        std::string savepath = "")
    {
        int cnt = 0;
        auto &matches = results;
        matches.assign(features1.size(), -1);
        // std::vector<int> matches(features1.size(), -1);
        std::vector<int> matchesDist(features1.size(), -1);

        //// debug
        //std::vector<Eigen::Vector4d> linePoints(features2.size());
        //int downTimes = 1;
        //int downTimesTmp = downTimes;
        //cv::Mat mmatch(img1.rows / downTimes, (img1.cols + img2.cols) / downTimes, CV_8UC1);
        //cv::Rect rect1(0, 0, img1.cols / downTimes, img1.rows / downTimes);
        //cv::Rect rect2(img2.cols / downTimes, 0, img1.cols / downTimes, img1.rows / downTimes);
        //cv::Mat imgDown, imgDown2;
        //imgDown = img1.clone();
        //imgDown2 = img2.clone();
        //if (downTimesTmp != 1)
        //{
        //    while (downTimesTmp / 2 != 1)
        //    {
        //        cv::pyrDown(imgDown, imgDown);
        //        cv::pyrDown(imgDown2, imgDown2);
        //        downTimesTmp /= 2;
        //    }
        //}
        //imgDown.copyTo(mmatch(rect1));
        //imgDown2.copyTo(mmatch(rect2));

        // compute F matrix
        Eigen::Matrix3d t_mul;
        t_mul << 0, -1 * trans(2), trans(1),
            trans(2), 0, -1 * trans(0),
            -1 * trans(1), trans(0), 0;
        Eigen::Matrix3d F = K1.inverse().transpose() * t_mul * rota * K1.inverse();

        for (size_t i = 0; i < features2.size(); ++i)
        {
            Eigen::Vector3d p2(features2[i].pt.x, features2[i].pt.y, 1);
            const cv::Mat &des2 = desc2.row(i);

            // compute line
            Eigen::Vector3d line = F * p2;

            // get ids
            std::vector<size_t> ids;
            grid1->getKeypointIdsAround(line, ids);
            if (ids.empty())
            {
                // std::cout<<"not find ids around line, point("<<p2(0)<<","<<p2(1)<<")"<<std::endl;
                continue;
            }

            // //debug
            // cv::Mat matLine = mmatch.clone();
            // // // std::cout<<"ids around line, point("<<p2(0)<<","<<p2(1)<<")"<<std::endl;

            // match
            int best_id = -1;
            int best_dist = 128 * 255;
            int second_id = -1;
            int second_dist = 128 * 255;
            for (size_t j = 0; j < ids.size(); ++j)
            {

                // // //debug
                // cv::Point2d p1(features->at(ids[j]).x/downTimes, features->at(ids[j]).y/downTimes);
                // cv::circle(matLine, p1, 1, cv::Scalar(255,0,0));

                // compute distance
                const cv::Mat &des1 = desc1.row(ids[j]);
                const int dist = DescriptorDistance(des1, des2);

                // judge
                if (dist < best_dist)
                {
                    second_id = (int)best_id;
                    second_dist = best_dist;
                    best_id = (int)ids[j];
                    best_dist = dist;
                }
                else if (dist < second_dist)
                {
                    second_id = (int)ids[j];
                    second_dist = dist;
                }
            }

            // //debug
            // // std::cout<<"best dist:"<<best_dist<<" second dist:"<<second_dist<<std::endl;
            // cv::line(matLine, cv::Point(linePoints[i](0)/downTimes,linePoints[i](1)/downTimes),
            //                     cv::Point(linePoints[i](2)/downTimes,linePoints[i](3)/downTimes),
            //                     cv::Scalar(0,255,255),1);
            // cv::Point2d p((features2->at(i).x+img2.cols)/downTimes, features2->at(i).y/downTimes);
            // cv::circle(matLine, p, 3, cv::Scalar(255,0,255));
            // // cv::imshow("F line", matLine);
            // // cv::waitKey();

            // record
            if (best_dist > th || second_dist * nnTh < best_dist)
                continue;
            if ((matches[best_id] != -1) && (matchesDist[best_id] < best_dist))
            {
                continue;
            }
            matches[best_id] = i;
            matchesDist[best_id] = best_dist;

            // cv::Point2d pbest(features->at(best_id).x/downTimes, features->at(best_id).y/downTimes);
            // cv::circle(matLine, pbest, 3, cv::Scalar(255,0,255));
            // // cv::imshow("F line", matLine);
            // // cv::waitKey();
        }

        for (size_t ind = 0; ind < matches.size(); ++ind)
        {
            if (matches[ind] == -1)
                continue;
            ++cnt;
            // results.push_back({ ind, matches[ind] });
        }

        //// debug
        ////  for(size_t ind=0;ind<features2->size();++ind){
        ////      // std::cout<<"draw line:"<<ind<<std::endl;
        ////      cv::Mat matLine = mmatch.clone();
        ////      cv::line(matLine, cv::Point(linePoints[ind](0)/downTimes,linePoints[ind](1)/downTimes),
        ////                      cv::Point(linePoints[ind](2)/downTimes,linePoints[ind](3)/downTimes),
        ////                      cv::Scalar(0,255,255),1);
        ////      cv::Point2d p((features2->at(ind).x+img2.cols)/downTimes, features2->at(ind).y/downTimes);
        ////      cv::circle(matLine, p, 3, cv::Scalar(255,0,255));
        ////      cv::imshow("F line", matLine);
        ////      cv::waitKey();
        ////  }
        //for (size_t ind = 0; ind < matches.size(); ++ind)
        //{
        //    if (matches[ind] == -1)
        //        continue;
        //    cv::Point2d p1(features1.at(ind).pt.x / downTimes, features1.at(ind).pt.y / downTimes);
        //    cv::Point2d p2((features2.at(matches[ind]).pt.x + img2.cols) / downTimes, features2.at(matches[ind]).pt.y / downTimes);
        //    cv::circle(mmatch, p1, 3, cv::Scalar(255));
        //    cv::circle(mmatch, p2, 3, cv::Scalar(255));
        //    cv::line(mmatch, p1, p2, cv::Scalar(255), 1);
        //}
        //// cv::namedWindow("test match", cv::WINDOW_NORMAL);
        //// cv::imshow("test match", mmatch);
        //// cv::waitKey(1);
        //if (savepath != "")
        //{
        //    // std::string savepath = "/home/yang/imgdebug/match/" + std::to_string(imgi++) + ".png";
        //    cv::imwrite(savepath, mmatch);
        //}

        return cnt;
    }
    static int matchByBoW(
        const std::vector<cv::KeyPoint>& _kps1, const std::vector<cv::KeyPoint>& _kps2,
        const cv::Mat& _desc1, const cv::Mat& _desc2,
        const DBoW3::FeatureVector& _fv1, const DBoW3::FeatureVector& _fv2,
        std::vector<int>& _matches2to1,
        SLAM_LYJ::Pose3D& _Tcw1, SLAM_LYJ::Pose3D& _Tcw2,
        std::vector<Eigen::Vector3f>& _P3Ds1, std::vector<Eigen::Vector3f>& _P3Ds2, const float& _squareDistTh
    )
    {
        bool mbCheckOrientation = true;
        float mfNNratio = 0.6;
        int size1 = _kps1.size();
        _matches2to1.assign(size1, -1);
        int nmatches = 0;
        std::vector<int> rotHist[HISTO_LENGTH];
        for (int i = 0; i < HISTO_LENGTH; i++)
            rotHist[i].reserve(500);
        const float factor = 1.0f / HISTO_LENGTH;

        // We perform the matching over ORB that belong to the same vocabulary node (at a certain level)
        DBoW3::FeatureVector::const_iterator KFit = _fv2.begin();
        DBoW3::FeatureVector::const_iterator Fit = _fv1.begin();
        DBoW3::FeatureVector::const_iterator KFend = _fv2.end();
        DBoW3::FeatureVector::const_iterator Fend = _fv1.end();

        SLAM_LYJ::Pose3D Twc1 = _Tcw1.inversed();
        SLAM_LYJ::Pose3D Twc2 = _Tcw2.inversed();
        Eigen::Vector3f Pw1;
        Eigen::Vector3f Pw2;

        while (KFit != KFend && Fit != Fend)
        {
            if (KFit->first == Fit->first)
            {
                const std::vector<unsigned int> vIndicesKF = KFit->second;
                const std::vector<unsigned int> vIndicesF = Fit->second;

                for (size_t iKF = 0; iKF < vIndicesKF.size(); iKF++)
                {
                    const unsigned int realIdxKF = vIndicesKF[iKF];
                    const cv::Mat& dKF = _desc2.row(realIdxKF);

                    if (!_P3Ds1.empty() && !_P3Ds2.empty() && realIdxKF < _P3Ds2.size() && _P3Ds2[realIdxKF](2) > 0)
                        Pw2 = Twc2 * _P3Ds2[realIdxKF];

                    int bestDist1 = 256;
                    int bestIdxF = -1;
                    int bestDist2 = 256;
                    for (size_t iF = 0; iF < vIndicesF.size(); iF++)
                    {
                        const unsigned int realIdxF = vIndicesF[iF];
                        if (_matches2to1[realIdxF] != -1)
                            continue;
                        if (!_P3Ds1.empty() && !_P3Ds2.empty() && realIdxF < _P3Ds1.size() && _P3Ds1[realIdxF](2) > 0 && realIdxKF < _P3Ds2.size() && _P3Ds2[realIdxKF](2) > 0)
                        {
                            Pw1 = Twc1 * _P3Ds1[realIdxF];
                            //std::cout << (Pw1 - Pw2).squaredNorm() << std::endl;
                            if ((Pw1 - Pw2).squaredNorm() > _squareDistTh)
                                continue;
                        }
                        const cv::Mat& dF = _desc1.row(realIdxF);
                        const int dist = DescriptorDistance(dKF, dF);
                        if (dist < bestDist1)
                        {
                            bestDist2 = bestDist1;
                            bestDist1 = dist;
                            bestIdxF = realIdxF;
                        }
                        else if (dist < bestDist2)
                        {
                            bestDist2 = dist;
                        }

                    }

                    if (bestDist1 <= TH_LOW)
                    {
                        if (static_cast<float>(bestDist1) < mfNNratio * static_cast<float>(bestDist2))
                        {
                            _matches2to1[bestIdxF] = realIdxKF;


                            if (mbCheckOrientation)
                            {
                                const cv::KeyPoint& kp = _kps2[realIdxKF];
                                const cv::KeyPoint& Fkp = _kps1[bestIdxF];
                                float rot = kp.angle - Fkp.angle;
                                if (rot < 0.0)
                                    rot += 360.0f;
                                int bin = round(rot * factor);
                                if (bin == HISTO_LENGTH)
                                    bin = 0;
                                assert(bin >= 0 && bin < HISTO_LENGTH);
                                rotHist[bin].push_back(bestIdxF);
                            }
                            nmatches++;
                        }
                    }

                }

                KFit++;
                Fit++;
            }
            else if (KFit->first < Fit->first)
            {
                KFit = _fv2.lower_bound(Fit->first);
            }
            else
            {
                Fit = _fv1.lower_bound(KFit->first);
            }
        }

        if (mbCheckOrientation)
        {
            int ind1 = -1;
            int ind2 = -1;
            int ind3 = -1;

            ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

            for (int i = 0; i < HISTO_LENGTH; i++)
            {
                if (i == ind1 || i == ind2 || i == ind3)
                    continue;
                for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++)
                {
                    _matches2to1[rotHist[i][j]] = -1;
                    nmatches--;
                }
            }
        }

        return nmatches;
    }


    PointMatcher::PointMatcher(Option _opt) : MatcherAbr(MatcherAbr::TYPE::POINT), opt_(_opt)
    {
        if (_opt.mode <= 0)
        {
        }
        else if (_opt.mode <= 6)
        {
            descMatcher_ = cv::DescriptorMatcher::create(_opt.mode);
            bfMatcher_ = std::make_shared<cv::BFMatcher>(cv::NORM_HAMMING);
        }
        else
        {
        }
    }

    PointMatcher::~PointMatcher()
    {
    }

    int PointMatcher::match(ImageExtractData *const _frame1, ImageExtractData *const _frame2, ImageMatchData *const _result)
    {
        if (!_result || !_frame1 || !_frame2 || !_result->usePointMatch || !_frame1->usePointFeature || !_frame2->usePointFeature)
            return -1;
        int kpSize1 = _frame1->kps_.size();
        int kpSize2 = _frame2->kps_.size();
        _result->match2to1P.assign(kpSize1, -1);
        int cnt = 0;
        if (opt_.mode <= 0)
        {
            cnt = matchByBoW(_frame1->kps_, _frame2->kps_, _frame1->descriptors_, _frame2->descriptors_, _frame1->featureVec, _frame2->featureVec, _result->match2to1P, _frame1->Tcw, _frame2->Tcw, _frame1->kp3Ds_, _frame2->kp3Ds_, opt_.squareDThInMesh);
        }
        else if (opt_.mode <= 6)
        {
            std::vector<cv::DMatch> matches;
            bfMatcher_->match(_frame1->descriptors_, _frame2->descriptors_, matches);
            for (size_t i = 0; i < matches.size(); ++i)
                _result->match2to1P[matches[i].queryIdx] = matches[i].trainIdx;
        }
        else if (opt_.mode == 7)
        {
            SLAM_LYJ::Pose3D T12 = _frame1->Tcw * _frame2->Tcw.inversed();
            // std::vector<std::pair<int, int>> rets;
            //std::string savepath = "";
            //if (_result->debugPath != "")
            //    savepath = _result->debugPath + std::to_string(_frame1->id) + "_" + std::to_string(_frame2->id) + ".png";
            cnt = matchByF(_frame1->cam->getK(),
                _frame1->kps_, _frame2->kps_,
                _frame1->descriptors_, _frame2->descriptors_,
                _frame1->featureGrid_.get(),
                T12.getR(), T12.gett(),
                _result->match2to1P, 50, 0.6);
                //,
                //     _frame1->img, _frame2->img,
                //     savepath);
        }
        else
        {
            printf("match mode not support!\n");
        }

        // fundamental
        if (cnt > 0 && opt_.check)
        {
            std::vector<cv::KeyPoint>& kps1 = _frame1->kps_;
            std::vector<cv::KeyPoint>& kps2 = _frame2->kps_;
            for (int it = 0; it < 1; ++it) {
                std::vector<Eigen::Vector2i> matches2RANSAC;
                matches2RANSAC.reserve(cnt);
                for (int i = 0; i < _result->match2to1P.size(); ++i)
                {
                    if (_result->match2to1P[i] == -1)
                        continue;
                    matches2RANSAC.push_back(Eigen::Vector2i(i, _result->match2to1P[i]));
                }
                //RANSACFundamental ransacF(&kps1, &kps2, matches2RANSAC, 0.05, 0.5, 8);
                //std::vector<float> errs;
                //std::vector<bool> inliners;
                //Eigen::Matrix3f F21;
                //std::vector<int> bestSample;
                //double ratio = ransacF.run(cnt, errs, inliners, F21, bestSample);
                RANSACHomography ransacH(&kps1, &kps2, matches2RANSAC, 30, 0.5, 4);
                std::vector<float> errs;
                std::vector<bool> inliners;
                Eigen::Matrix3f H21;
                std::vector<int> bestSample;
                double ratio = ransacH.run(cnt, errs, inliners, H21, bestSample);
                _result->match2to1P.assign(kpSize1, -1);
                for (int i = 0; i < inliners.size(); ++i)
                {
                    if (inliners[i])
                        _result->match2to1P[matches2RANSAC[i](0)] = matches2RANSAC[i](1);
                    else
                        --cnt;
                }
            }

            //std::vector<cv::Point2f> ps1, ps2;
            //std::vector<int> id1s, id2s;
            //for (size_t i = 0; i < kpSize1; ++i) {
            //    if (_result->match2to1P[i] == -1)
            //        continue;
            //    ps1.push_back(_frame1->kps_[i].pt);
            //    ps2.push_back(_frame2->kps_[_result->match2to1P[i]].pt);
            //    id1s.push_back(i);
            //    id2s.push_back(_result->match2to1P[i]);
            //}
            //cv::Mat binliners;
            //cv::Mat F = cv::findFundamentalMat(ps1, ps2, binliners, 8, 1.0);
            //cnt = 0;
            //_result->match2to1P.assign(kpSize1, -1);
            //for (int i = 0; i < binliners.rows; ++i) {
            //    if (binliners.at<uchar>(i, 0) == 0)
            //        continue;
            //    _result->match2to1P[id1s[i]] = id2s[i];
            //    ++cnt;
            //}
        }
        _result->pointMatchSize = cnt;
        return cnt;
    }

}