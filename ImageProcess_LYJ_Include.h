#ifndef IMAGEPROCESS_LYJ_INCLUDE_H
#define IMAGEPROCESS_LYJ_INCLUDE_H


#include "ImageProcess_LYJ_Defines.h"
#include <stdio.h>



namespace ImageProcess_LYJ
{


IMAGEPROCESS_LYJ_API void print_ImageProcess_LYJ_Test();

IMAGEPROCESS_LYJ_API void extractORBFeature(ImageExtractData* _frame);
IMAGEPROCESS_LYJ_API int matchORBFeature(ImageExtractData* const _frame1, ImageExtractData* const _frame2, ImageMatchData* const _matchResult);


}

#endif//IMAGEPROCESS_LYJ_INCLUDE_H