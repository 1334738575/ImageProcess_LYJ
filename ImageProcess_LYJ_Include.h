#ifndef IMAGEPROCESS_LYJ_INCLUDE_H
#define IMAGEPROCESS_LYJ_INCLUDE_H


#include "ImageProcess_LYJ_Defines.h"
#include <stdio.h>



namespace ImageProcess_LYJ
{


	IMAGEPROCESS_LYJ_API void print_ImageProcess_LYJ_Test();
	IMAGEPROCESS_LYJ_API void testDBoW3(std::vector<cv::Mat>& features);

IMAGEPROCESS_LYJ_API void extractFeature(ImageExtractData* _frame, const ImageExtractOption& _opt= ImageExtractOption());
IMAGEPROCESS_LYJ_API int matchFeature(ImageExtractData* const _frame1, ImageExtractData* const _frame2, ImageMatchData* const _matchResult, const ImageMatchOption& _opt= ImageMatchOption());
IMAGEPROCESS_LYJ_API bool reconstructTwo(ImageExtractData* const _frame1, ImageExtractData* const _frame2, ImageMatchData* const _matchResult, ImageTriangleData* const _triangleResult, const ImageTriangleOption& _opt= ImageTriangleOption());


}

#endif//IMAGEPROCESS_LYJ_INCLUDE_H