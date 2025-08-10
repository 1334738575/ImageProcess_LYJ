#ifndef IMAGEPROCESS_LYJ_EXTRACTORABR_H
#define IMAGEPROCESS_LYJ_EXTRACTORABR_H

#include "ImageProcess_LYJ_Defines.h"
#include <opencv2/opencv.hpp>

namespace ImageProcess_LYJ{

class ExtractorAbr
{
public:
	typedef void* ExtractorOutput;
	enum class TYPE
	{
		OBR = 0,
		SIFT,
		LSD,
		CANNY
	};
	ExtractorAbr(const TYPE _type) :type_(_type) {};
	~ExtractorAbr() {};

	//for visual feature����ʱ���ã�ͼƬ������ܶ��𻵣���Ҫ��һ���ж�ԭ���ձ顢�ɻ��ӡ��ƶ�Ӳ�̣���Mat�����е�����
	virtual void extract(cv::Mat _img, ExtractorOutput _output) {
		std::cout << "This have been discaeded!" << std::endl;
	}

	virtual void extract(cv::Mat _img, ImageExtractData* _frame) = 0;
	TYPE getType() { return type_; }
protected:
	TYPE  type_;
};


}

#endif //IMAGEPROCESS_LYJ_EXTRACTORABR_H