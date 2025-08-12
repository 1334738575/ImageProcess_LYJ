#include "PointMatcher.h"

namespace ImageProcess_LYJ
{

    // static int imgi = 0;
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

        // debug
        std::vector<Eigen::Vector4d> linePoints(features2.size());
        int downTimes = 1;
        int downTimesTmp = downTimes;
        cv::Mat mmatch(img1.rows / downTimes, (img1.cols + img2.cols) / downTimes, CV_8UC1);
        cv::Rect rect1(0, 0, img1.cols / downTimes, img1.rows / downTimes);
        cv::Rect rect2(img2.cols / downTimes, 0, img1.cols / downTimes, img1.rows / downTimes);
        cv::Mat imgDown, imgDown2;
        imgDown = img1.clone();
        imgDown2 = img2.clone();
        if (downTimesTmp != 1)
        {
            while (downTimesTmp / 2 != 1)
            {
                cv::pyrDown(imgDown, imgDown);
                cv::pyrDown(imgDown2, imgDown2);
                downTimesTmp /= 2;
            }
        }
        imgDown.copyTo(mmatch(rect1));
        imgDown2.copyTo(mmatch(rect2));

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

        // debug
        //  for(size_t ind=0;ind<features2->size();++ind){
        //      // std::cout<<"draw line:"<<ind<<std::endl;
        //      cv::Mat matLine = mmatch.clone();
        //      cv::line(matLine, cv::Point(linePoints[ind](0)/downTimes,linePoints[ind](1)/downTimes),
        //                      cv::Point(linePoints[ind](2)/downTimes,linePoints[ind](3)/downTimes),
        //                      cv::Scalar(0,255,255),1);
        //      cv::Point2d p((features2->at(ind).x+img2.cols)/downTimes, features2->at(ind).y/downTimes);
        //      cv::circle(matLine, p, 3, cv::Scalar(255,0,255));
        //      cv::imshow("F line", matLine);
        //      cv::waitKey();
        //  }
        for (size_t ind = 0; ind < matches.size(); ++ind)
        {
            if (matches[ind] == -1)
                continue;
            cv::Point2d p1(features1.at(ind).pt.x / downTimes, features1.at(ind).pt.y / downTimes);
            cv::Point2d p2((features2.at(matches[ind]).pt.x + img2.cols) / downTimes, features2.at(matches[ind]).pt.y / downTimes);
            cv::circle(mmatch, p1, 3, cv::Scalar(255));
            cv::circle(mmatch, p2, 3, cv::Scalar(255));
            cv::line(mmatch, p1, p2, cv::Scalar(255), 1);
        }
        // cv::namedWindow("test match", cv::WINDOW_NORMAL);
        // cv::imshow("test match", mmatch);
        // cv::waitKey(1);
        if (savepath != "")
        {
            // std::string savepath = "/home/yang/imgdebug/match/" + std::to_string(imgi++) + ".png";
            cv::imwrite(savepath, mmatch);
        }

        return cnt;
    }

    PointMatcher::PointMatcher(Option _opt) : MatcherAbr(MatcherAbr::TYPE::POINT), opt_(_opt)
    {
        if (_opt.mode <= 0)
        {
        }
        else if (_opt.mode <= 6)
        {
            descMatcher_ = cv::DescriptorMatcher::create(_opt.mode);
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
        if (opt_.mode <= 0)
        {
            return 0;
        }
        else if (opt_.mode <= 6)
        {
            std::vector<cv::DMatch> matches;
            descMatcher_->match(_frame1->descriptors_, _frame2->descriptors_, matches);
            for (size_t i = 0; i < matches.size(); ++i)
            {
                _result->match2to1P[matches[i].queryIdx] = matches[i].trainIdx;
            }
        }
        // fundamental
        else if (opt_.mode == 7)
        {
            SLAM_LYJ::Pose3D T12 = _frame1->Tcw * _frame2->Tcw.inversed();
            // std::vector<std::pair<int, int>> rets;
            std::string savepath = "";
            if (_result->debugPath != "")
                savepath = _result->debugPath + std::to_string(frame.id) + "_" + std::to_string(frame2.id) + ".png";
            matchByF(_frame1->cam->getK(),
                     _frame1->kps_, _frame2->kps_,
                     _frame1->descriptors_, _frame2->descriptors_,
                     _frame1->featureGrid_.get(),
                     T12.getR(), T12.gett(),
                     _result->match2to1P, 50, 0.6,
                     _frame1->img, _frame2->img,
                     savepath);
        }
        else
        {
            printf("match mode not support!\n");
        }
        return 0;
    }

}