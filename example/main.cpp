#include <iostream>
#include <ImageProcess_LYJ_Defines.h>
#include <ImageProcess_LYJ_Include.h>
// #include <thi>

int main()
{
    ImageProcess_LYJ::print_ImageProcess_LYJ_Test();

    std::vector<double> camd = {2048, 2048, 765.955, 766.549, 1024, 1024};
    SLAM_LYJ::PinholeCmera cam(camd);

    std::string dataPath = "D:/tmp/images/";
    std::string dataPath2 = "D:/tmp/texture_data/RT_";
    int id1 = 7;
    int id2 = 13;
    auto funcReadTcw = [](const std::string &_file, SLAM_LYJ::Pose3D &Tcw)
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
    ImageProcess_LYJ::extractORBFeature(&frame);
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
    ImageProcess_LYJ::extractORBFeature(&frame2);
    std::cout << "key point2 size: " << frame2.kps_.size() << std::endl;
    // cv::drawKeypoints(img2, frame2.kps_, img2, cv::Scalar(255, 0, 0));
    // cv::imshow("keypoints2", img2);
    // cv::waitKey();

    ImageProcess_LYJ::ImageMatchData matchResult;
    matchResult.usePointMatch = true;
    matchResult.debugPath = "D:/tmp/imageProcess/match/";
    ImageProcess_LYJ::matchORBFeature(&frame, &frame2, &matchResult);

    //ImageProcess_LYJ::

    return 0;
}