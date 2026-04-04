#ifndef IMAGEPROCESS_LYJ_FEATURE_GRID_H
#define IMAGEPROCESS_LYJ_FEATURE_GRID_H

#include <opencv2/opencv.hpp>
#include <Eigen/Core>

#ifdef WIN32
#ifdef _MSC_VER
#define FEATUREGRID_LYJ_API __declspec(dllexport)
#else
#define FEATUREGRID_LYJ_API
#endif
#else
#define FEATUREGRID_LYJ_API
#endif


namespace ImageProcess_LYJ
{

	class FEATUREGRID_LYJ_API FeatureGrid {
	public:
		FeatureGrid();
	    //resolution=20
	    FeatureGrid(const int w, const int h, const int resolution, const std::vector<cv::KeyPoint>& features);
	
		void getKeypointIdsAround(const Eigen::Vector3d& line, std::vector<size_t>& ids);
		void getKeypointIdsAround(const float& x, const float& y, std::vector<size_t>& ids);
	
	    inline int& getResolution() {
	        return resolution;
	    }
	public:
	    int resolution = 1;
	    int max_row = 0;
	    int max_col = 0;
	    std::unordered_map<int, std::vector<size_t>> grid;
	};


	class FEATUREGRID_LYJ_API FeatureGridAbr
	{
	public:
		FeatureGridAbr(const int& _maxW, const int& _maxH, const int& _resolu)
			:maxW_(_maxW), maxH_(_maxH), resolu_(_resolu)
		{
			gridW_ = (maxW_ + resolu_ - 1) / resolu_;
			gridH_ = (maxH_ + resolu_ - 1) / resolu_;
		};
		~FeatureGridAbr() {};

		virtual void init(const std::vector<cv::KeyPoint>& _kps) = 0;
		virtual bool isIndInGrid(const int& _cGrid, const int& _rGrid)
		{
			if (_cGrid < 0 || _rGrid < 0 || _cGrid >= gridW_ || _rGrid >= gridH_)
				return false;
			return true;
		}
		virtual bool getIndGrid(const float& _u, const float& _v, int& _cGrid, int& _rGrid)
		{
			_cGrid = int(_u) / resolu_;
			_rGrid = int(_v) / resolu_;
			return isIndInGrid(_cGrid, _rGrid);
		}
		virtual void getKpIndInCell(const int& _cGrid, const int& _rGrid, short* _kpIndSt, int& _kpIndSz) = 0;

	protected:
		int maxW_ = 0;
		int maxH_ = 0;
		int resolu_ = 1;
		int gridW_ = 0;
		int gridH_ = 0;

	};
	class FEATUREGRID_LYJ_API FeatureGridConst: public FeatureGridAbr
	{
	public:
		FeatureGridConst();
		FeatureGridConst(const std::vector<cv::KeyPoint>& _kps);
		~FeatureGridConst();

		virtual void init(const std::vector<cv::KeyPoint>& _kps);
		virtual void getKpIndInCell(const int& _cGrid, const int& _rGrid, short* _kpIndSt, int& _kpIndSz);

	protected:
		const int maxKpSzInCell_ = 128;
		std::vector<short> cellDatas_;
		std::vector<char> cellIndSz_;
	};
	class FEATUREGRID_LYJ_API FeatureGridConstCom : public FeatureGridAbr
	{
	public:
		FeatureGridConstCom();
		FeatureGridConstCom(const std::vector<cv::KeyPoint>& _kps);
		~FeatureGridConstCom();

		virtual void init(const std::vector<cv::KeyPoint>& _kps);
		virtual void getKpIndInCell(const int& _cGrid, const int& _rGrid, short* _kpIndSt, int& _kpIndSz);

	private:
		const int maxKpSz_ = 8192;
		std::vector<short> cellDatas_;
		std::vector<short> cellInd_;
	};

	class FEATUREGRID_LYJ_API FeatureGridFromORB
	{
	public:
		FeatureGridFromORB() = delete;
		FeatureGridFromORB(const int _w, const int _h, std::vector<cv::KeyPoint>* _kps, const int _resolution=20);
		~FeatureGridFromORB() {};

		std::vector<size_t> GetFeaturesInArea(const float& x, const float& y, const float& r) const;
		bool PosInGrid(const cv::KeyPoint& kp, int& posX, int& posY);

	private:
		std::vector<std::vector<std::vector<std::size_t>>> mGrid_;
		const int w_;
		int wGrid_;
		const int h_;
		int hGrid_;
		const int resolution_;
		float invResolution_;
		std::vector<cv::KeyPoint>* kps_ = nullptr;
	};
}


#endif