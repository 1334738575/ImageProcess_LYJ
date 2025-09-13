#include "ImageProcess_LYJ_Defines.h"
        


namespace ImageProcess_LYJ
{
        
	IMAGEPROCESS_LYJ_API void site2Key(const uint32_t& _s1, const uint32_t& _s2, uint64_t& _k)
	{
		if (_s1 < _s2)
			_k = ((uint64_t)_s1 << 32) | (uint64_t)_s2;
		else
			_k = ((uint64_t)_s2 << 32) | (uint64_t)_s1;
	}
	IMAGEPROCESS_LYJ_API void key2Site(const uint64_t& _k, uint32_t& _s1, uint32_t& _s2)
	{
		_s1 = (uint32_t)(_k >> 32);
		_s2 = (uint32_t)(_k & 0xFFFFFFFF);
	}
        
}