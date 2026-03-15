#include <iostream>
#include <ImageProcess_LYJ_Defines.h>
#include <ImageProcess_LYJ_Include.h>

#include <STLPlus/include/file_system.h>
#include <common/ThreadPool.h>


void test1()
{
    std::vector<double> camd = { 765.955, 766.549, 1024, 1024 };
    COMMON_LYJ::PinholeCamera cam(2048, 2048, camd);

    std::string dataPath = "D:/tmp/images/";
    std::string dataPath2 = "D:/tmp/texture_data/RT_";
    int id1 = 7;
    int id2 = 13;
    auto funcReadTcw = [](const std::string& _file, COMMON_LYJ::Pose3D& Tcw)
        {
            std::ifstream f(_file);
            if (!f.is_open())
            {
                std::cout << "read pose failed!" << std::endl;
                return;
            }
            f >> Tcw.getR()(0, 0) >> Tcw.getR()(0, 1) >> Tcw.getR()(0, 2) >> Tcw.gett()(0) >> Tcw.getR()(1, 0) >> Tcw.getR()(1, 1) >> Tcw.getR()(1, 2) >> Tcw.gett()(1) >> Tcw.getR()(2, 0) >> Tcw.getR()(2, 1) >> Tcw.getR()(2, 2) >> Tcw.gett()(2);
            f.close();
        };

    cv::Mat img = cv::imread(dataPath + std::to_string(id1) + ".png", 0);
    cv::pyrDown(img, img);
    // cv::imshow("image", img);
    // cv::waitKey();
    ImageProcess_LYJ::ImageExtractData frame;
    frame.id = id1;
    frame.cam = &cam;
    frame.img = img;
    funcReadTcw(dataPath2 + std::to_string(id1) + ".txt", frame.Tcw);
    ImageProcess_LYJ::extractFeature(&frame);
    std::cout << "key point size: " << frame.kps_.size() << std::endl;
    // cv::drawKeypoints(img, frame.kps_, img, cv::Scalar(255, 0, 0));
    // cv::imshow("keypoints", img);
    // cv::waitKey();
    cv::Mat img2 = cv::imread(dataPath + std::to_string(id2) + ".png", 0);
    cv::pyrDown(img2, img2);
    ImageProcess_LYJ::ImageExtractData frame2;
    frame2.id = id2;
    frame2.cam = &cam;
    frame2.img = img2;
    funcReadTcw(dataPath2 + std::to_string(id2) + ".txt", frame2.Tcw);
    ImageProcess_LYJ::extractFeature(&frame2);
    std::cout << "key point2 size: " << frame2.kps_.size() << std::endl;
    // cv::drawKeypoints(img2, frame2.kps_, img2, cv::Scalar(255, 0, 0));
    // cv::imshow("keypoints2", img2);
    // cv::waitKey();

    ImageProcess_LYJ::ImageMatchData matchResult;
    matchResult.usePointMatch = true;
    matchResult.debugPath = "D:/tmp/imageProcess/match/";
    ImageProcess_LYJ::matchFeature(&frame, &frame2, &matchResult);

    std::vector<cv::Mat> features;
    features.push_back(frame.descriptors_);
    features.push_back(frame2.descriptors_);
    ImageProcess_LYJ::testDBoW3(features);
}
void test2()
{
    using namespace ImageProcess_LYJ;
    std::string imgPath = "D:/tmp/colmapData/mask2/images/";
    std::vector<std::string> imgFiles = stlplus::folder_files(imgPath);
    int imgSz = imgFiles.size();
    std::vector<ImageExtractData> frames(imgSz);
    int thdNum = std::thread::hardware_concurrency();
    COMMON_LYJ::ThreadPool thdPl(thdNum);
    ImageExtractOption exOpt;
    auto funcExtract = [&](uint64_t _s, uint64_t _e) {
        for (int i = _s; i < _e; ++i)
        {
            frames[i].path = imgPath + imgFiles[i];
            cv::Mat img = cv::imread(frames[i].path);
            extractFeature(&frames[i], exOpt);
        }
    };
    thdPl.process(funcExtract, 0, imgSz);

    std::vector<ImageMatchData> matchDatas;
    ImagePairOption pairOpt;
    pairOpt.useBF = false;
    pairOpt.useVoc = true;
    pairOpt.vocPathVoc = "D:/SLAM_LYJ/Reference/ORB_SLAM3-master/Vocabulary/ORBvoc.txt";
    pairGenerate(frames, matchDatas, pairOpt);

    int mthSz = matchDatas.size();
    ImageMatchOption mthOpt;
    mthOpt.usePointMatch = true;
    mthOpt.pointMatchMode = 6;
    mthOpt.bTriangle = false;
    auto funcMatch = [&](uint64_t _s, uint64_t _e) {
        for (int i = _s; i < _e; ++i)
        {
            ImageMatchData* matchData = &matchDatas[i];
            int id1 = matchData->id1;
            int id2 = matchData->id2;
            matchFeature(&frames[id1], &frames[id2], matchData, mthOpt);
        }
    };
    thdPl.process(funcMatch, 0, mthSz);

}

int main()
{
    ImageProcess_LYJ::print_ImageProcess_LYJ_Test();

    test1();

    return 0;
}