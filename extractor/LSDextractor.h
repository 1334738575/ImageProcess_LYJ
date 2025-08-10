#ifndef SLAM_LYJ_LSDEXTRACTOR_H
#define SLAM_LYJ_LSDEXTRACTOR_H

#include "extractorAbr.h"
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/line_descriptor.hpp>

namespace ImageProcess_LYJ{


class LSDExtractor : public ExtractorAbr
{
public:
	struct Option
	{
		bool needDesc = false;
	};
	LSDExtractor(Option _opt);
	~LSDExtractor();


	// ͨ�� ExtractorAbr �̳�
	void extract(cv::Mat _img, ImageExtractData* _frame) override;

	static void convertKeyLine2CVVec4f(const std::vector<cv::line_descriptor::KeyLine>& _keyLines, std::vector<cv::Vec4f>& _lines);
	static void convertCVVec4f2KeyLine(const std::vector<cv::Vec4f>& _lines, std::vector<cv::line_descriptor::KeyLine>& _keyLines);


private:
	cv::Ptr<cv::LineSegmentDetector> lsd_;
	cv::Ptr<cv::line_descriptor::LSDDetector> lsdKeyLine_;
	cv::Ptr<cv::line_descriptor::BinaryDescriptor> lineDescriptor_;
	Option opt_;
};


}

#endif //SLAM_LYJ_LSDEXTRACTOR_H