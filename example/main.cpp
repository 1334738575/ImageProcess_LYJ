#include <iostream>
#include <ImageProcess_LYJ_Defines.h>
#include <ImageProcess_LYJ_Include.h>
//#include <thi>

        
int main()
{
    ImageProcess_LYJ::print_ImageProcess_LYJ_Test();

    std::vector<double> camd = { 2048, 2048, 765.955, 766.549, 1024, 1024 };
    SLAM_LYJ::PinholeCmera cam(camd);

    cv::Mat img = cv::imread("D:/tmp/images/7.png", 0);
    cv::pyrDown(img, img);
    //cv::imshow("image", img);
    //cv::waitKey();
    ImageProcess_LYJ::ImageExtractData frame;
    frame.cam = &cam;
    frame.img = img;
    ImageProcess_LYJ::extractORBFeature(&frame);
    std::cout << "key point size: " << frame.kps_.size() << std::endl;
    //cv::drawKeypoints(img, frame.kps_, img, cv::Scalar(255, 0, 0));
    //cv::imshow("keypoints", img);
    //cv::waitKey();
    cv::Mat img2 = cv::imread("D:/tmp/images/8.png", 0);
    cv::pyrDown(img2, img2);
    ImageProcess_LYJ::ImageExtractData frame2;
    frame2.Tcw.gett() = Eigen::Vector3d(0.1, 0.1, 0.1);
    frame2.cam = &cam;
    frame2.img = img2;
    ImageProcess_LYJ::extractORBFeature(&frame2);
    std::cout << "key point2 size: " << frame2.kps_.size() << std::endl;
    //cv::drawKeypoints(img2, frame2.kps_, img2, cv::Scalar(255, 0, 0));
    //cv::imshow("keypoints2", img2);
    //cv::waitKey();

    ImageProcess_LYJ::ImageMatchData matchResult;
	matchResult.usePointMatch = true;
    matchResult.debugPath = "D:/tmp/imageProcess/match/";
    ImageProcess_LYJ::matchORBFeature(&frame, &frame2, &matchResult);

    return 0;
}