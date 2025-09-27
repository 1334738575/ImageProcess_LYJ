#include "featureGrid.h"



namespace ImageProcess_LYJ
{
	FeatureGrid::FeatureGrid(const int w, const int h, const int resolution, const std::vector<cv::KeyPoint>& features) {
        this->resolution = resolution;
        max_row = w / resolution;
        max_col = h / resolution;
        for (size_t i = 0; i < features.size(); ++i) {
            int x = features[i].pt.x / resolution;
            int y = features[i].pt.y / resolution;
            grid[y * max_col + x].push_back(i);
        }
    }

    void FeatureGrid::getKeypointIdsAround(const Eigen::Vector3d& line, std::vector<size_t>& ids) {
        bool xtrend = true;
        Eigen::Vector3d line_resol = line;
        line_resol(2) /= resolution;
        if (abs(line_resol(0)) > abs(line_resol(1)))
            xtrend = false;
        if (xtrend) {
            double k = -1 * (line_resol(0)) / line_resol(1);
            double b = -1 * (line_resol(2)) / line_resol(1);
            for (int indx = 0; indx < max_col; ++indx) {
                int indy = k * indx + b;
                if (indy < 0) {
                    if (k > 0) continue;
                    else break;
                }
                else if (indy > max_row) {
                    if (k < 0) continue;
                    else break;
                }
                int starty = (indy - 1) > 0 ? indy - 1 : 0;
                int endy = (indy + 1) < max_row ? indy + 1 : max_row;
                for (int iy = starty; iy <= endy; ++iy) {
                    // std::cout<<"get ids in grid("<<indx<<","<<iy<<")"<<std::endl;
                    if (grid.count(iy * max_col + indx)) {
                        std::vector<size_t>& idsInGrid = grid[iy * max_col + indx];
                        ids.insert(ids.end(), idsInGrid.begin(), idsInGrid.end());
                    }
                }
            }
        }
        else {
            double k = -1 * (line_resol(1)) / line_resol(0);
            double b = -1 * (line_resol(2)) / line_resol(0);
            for (int indy = 0; indy < max_row; ++indy) {
                int indx = k * indy + b;
                if (indx < 0) {
                    if (k > 0) continue;
                    else break;
                }
                else if (indx > max_col) {
                    if (k < 0) continue;
                    else break;
                }
                int startx = (indx - 1) > 0 ? indx - 1 : 0;
                int endx = (indx + 1) < max_col ? indx + 1 : max_col;
                for (int ix = startx; ix <= endx; ++ix) {
                    if (grid.count(indy * max_col + ix)) {
                        // std::cout<<"get ids in grid("<<ix<<","<<indy<<")"<<std::endl;
                        std::vector<size_t>& idsInGrid = grid[indy * max_col + ix];
                        ids.insert(ids.end(), idsInGrid.begin(), idsInGrid.end());
                    }
                }
            }
        }
    }


    FeatureGridFromORB::FeatureGridFromORB(const int _w, const int _h, std::vector<cv::KeyPoint>* _kps, const int _resolution)
        :w_(_w), h_(_h), resolution_(_resolution), kps_(_kps)
    {
        invResolution_ = 1.0f / (float)resolution_;
        wGrid_ = ceil((float)w_ / (float)resolution_);
        hGrid_ = ceil((float)h_ / (float)resolution_);
		mGrid_.resize(wGrid_);
		for (int i = 0; i < wGrid_; i++)
			mGrid_[i].resize(hGrid_);
		const int N = kps_->size();
		for (int i = 0; i < N; i++)
		{
			const cv::KeyPoint& kp = kps_->at(i);
			int nGridPosX, nGridPosY;
			nGridPosX = (int)round((kp.pt.x) * invResolution_);
			nGridPosY = (int)round((kp.pt.y) * invResolution_);
			if (nGridPosX < 0 || nGridPosX >= wGrid_ || nGridPosY < 0 || nGridPosY >= hGrid_)
				continue;
			mGrid_[nGridPosX][nGridPosY].push_back(i);
		}
    }
    std::vector<size_t> FeatureGridFromORB::GetFeaturesInArea(const float& x, const float& y, const float& r) const
    {
        int N = kps_->size();
        std::vector<size_t> vIndices;
        vIndices.reserve(N);

        float factorX = r;
        float factorY = r;

        const int nMinCellX = std::max(0, (int)floor((x - factorX) * invResolution_));
        if (nMinCellX >= wGrid_)
            return vIndices;
        const int nMaxCellX = std::min((int)wGrid_ - 1, (int)ceil((x + factorX) * invResolution_));
        if (nMaxCellX < 0)
            return vIndices;
        const int nMinCellY = std::max(0, (int)floor((y - factorY) * invResolution_));
        if (nMinCellY >= hGrid_)
            return vIndices;
        const int nMaxCellY = std::min((int)hGrid_ - 1, (int)ceil((y + factorY) * invResolution_));
        if (nMaxCellY < 0)
            return vIndices;

        for (int ix = nMinCellX; ix <= nMaxCellX; ix++)
        {
            for (int iy = nMinCellY; iy <= nMaxCellY; iy++)
            {
                const std::vector<size_t> vCell = mGrid_[ix][iy];
                if (vCell.empty())
                    continue;
                for (size_t j = 0, jend = vCell.size(); j < jend; j++)
                {
                    const cv::KeyPoint& kpUn = kps_->at(vCell[j]);
                    const float distx = kpUn.pt.x - x;
                    const float disty = kpUn.pt.y - y;
                    if (fabs(distx) < factorX && fabs(disty) < factorY)
                        vIndices.push_back(vCell[j]);
                }
            }
        }

        return vIndices;
    }
    bool FeatureGridFromORB::PosInGrid(const cv::KeyPoint& kp, int& posX, int& posY)
    {
        posX = round((kp.pt.x) * invResolution_);
        posY = round((kp.pt.y) * invResolution_);

        //Keypoint's coordinates are undistorted, which could cause to go out of the image
        if (posX < 0 || posX >= wGrid_ || posY < 0 || posY >= hGrid_)
            return false;

        return true;
    }
}