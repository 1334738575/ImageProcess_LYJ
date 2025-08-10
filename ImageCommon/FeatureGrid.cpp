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
}