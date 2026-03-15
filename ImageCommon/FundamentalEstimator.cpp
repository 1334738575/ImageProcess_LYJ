#include "FundamentalEstimator.h"

namespace ImageProcess_LYJ
{
    FundamentalDecomposer::FundamentalDecomposer()
    {}
    FundamentalDecomposer::~FundamentalDecomposer()
    {}

    bool FundamentalDecomposer::decomposeEssentialMatrix(const cv::Mat& E, 
        const cv::Mat& K1, const cv::Mat& K2, 
        const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2, 
        Eigen::Matrix3d& R_out, Eigen::Vector3d& t_out)
    {
        if (E.rows != 3 || E.cols != 3 || pts1.empty() || pts1.size() != pts2.size()) {
            std::cerr << "输入参数错误！" << std::endl;
            return false;
        }

        // 1. OpenCV分解本质矩阵（得到4组可能的解）
        cv::Mat R1, R2;
        cv::Mat t;
        cv::decomposeEssentialMat(E, R1, R2, t); // 输出：Rs[0-3], ts[0-3]
        std::vector<cv::Mat> Rs(4);
        Rs[0] = R1;
        Rs[1] = R1;
        Rs[2] = R2;
        Rs[3] = R2;
        std::vector<cv::Mat> ts(4);
        ts[0] = t;
        ts[1] = -1 * t;
        ts[2] = t;
        ts[3] = -1 * t;

        // 2. 三角化验证：仅保留深度为正的解
        cv::Mat P1 = cv::Mat::eye(3, 4, CV_64F);  // 相机1位姿：P1 = [I | 0]
        P1 = K1 * P1;                              // 投影矩阵：P = K[R | t]

        for (int i = 0; i < 4; ++i) {
            // 构建相机2的投影矩阵 P2 = K2 [R | t]
            cv::Mat P2(3, 4, CV_64F);
            Rs[i].copyTo(P2.colRange(0, 3));
            ts[i].copyTo(P2.col(3));
            P2 = K2 * P2;

            // 三角化三维点
            std::vector<cv::Point3d> points3D;
            cv::triangulatePoints(P1, P2, pts1, pts2, points3D);

            // 检查深度是否为正（重投影到相机2，z>0）
            int positive_depth_count = 0;
            for (const auto& p3d : points3D) {
                // 齐次坐标转非齐次：(x/w, y/w, z/w)
                double w = p3d.z; // triangulatePoints输出的是(x,y,z,w)，需归一化
                if (w == 0) continue;
                double z = p3d.z / w;
                if (z > 1e-6) positive_depth_count++; // 深度阈值（避免浮点误差）
            }

            // 超过80%的点深度为正 → 正确解
            if (positive_depth_count > 0.8 * pts1.size()) {
                // 转换为Eigen格式
                for (int r = 0; r < 3; ++r) {
                    for (int c = 0; c < 3; ++c) {
                        R_out(r, c) = Rs[i].at<double>(r, c);
                    }
                    t_out(r) = ts[i].at<double>(r);
                }
                return true;
            }
        }

        std::cerr << "未找到有效RT解！" << std::endl;
        return false;
    }

    bool FundamentalDecomposer::decomposeFundamentalMatrix(const cv::Mat & F, const cv::Mat & K1, const cv::Mat & K2, const std::vector<cv::Point2f>&pts1, const std::vector<cv::Point2f>&pts2, Eigen::Matrix3d & R_out, Eigen::Vector3d & t_out)
    {
        // 1. F → E：E = K2^T * F * K1
        cv::Mat E = K2.t() * F * K1;

        // 2. 归一化特征点（像素坐标→归一化坐标）
        std::vector<cv::Point2f> pts1_norm, pts2_norm;
        cv::undistortPoints(pts1, pts1_norm, K1, cv::Mat());
        cv::undistortPoints(pts2, pts2_norm, K2, cv::Mat());

        // 3. 分解本质矩阵
        return decomposeEssentialMatrix(E, K1, K2, pts1_norm, pts2_norm, R_out, t_out);
    }




	int RANSACFundamental::calErr(const Eigen::Matrix3f& _mdl, const std::vector<int>& _datas, std::vector<float>& _errs, std::vector<bool>& _bInls)
	{
        if (m_kps1 == nullptr || m_kps2 == nullptr)
            return false;
        _errs.assign(_datas.size(), 0);
        _bInls.assign(_datas.size(), false);
        int cnt = 0;
        for (int i = 0; i < _datas.size(); ++i) {
            Eigen::Vector3f p1(m_kps1->at(m_matches[_datas[i]](0)).pt.x, m_kps1->at(m_matches[_datas[i]](0)).pt.y, 1);
            Eigen::Vector3f p2(m_kps2->at(m_matches[_datas[i]](1)).pt.x, m_kps2->at(m_matches[_datas[i]](1)).pt.y, 1);
            _errs[i] = std::abs(p2.transpose() * _mdl * p1);
            if (_errs[i] > m_errTh)
                continue;
            _bInls[i] =  true;
            ++cnt;
        }
		return cnt;
	}
    static void CenterAndNormalizeImagePoints(const std::vector<Eigen::Vector2d>& points,
        std::vector<Eigen::Vector2d>* normed_points,
        Eigen::Matrix3d* normed_from_orig) {
        const size_t num_points = points.size();
        if(num_points == 0)
            return;

        // Calculate centroid.
        Eigen::Vector2d centroid(0, 0);
        for (const Eigen::Vector2d& point : points) {
            centroid += point;
        }
        centroid /= num_points;

        // Root mean square distance to centroid of all points.
        double rms_mean_dist = 0;
        for (const Eigen::Vector2d& point : points) {
            rms_mean_dist += (point - centroid).squaredNorm();
        }
        rms_mean_dist = std::sqrt(rms_mean_dist / num_points);

        // Compose normalization matrix.
        const double norm_factor = std::sqrt(2.0) / rms_mean_dist;
        *normed_from_orig << norm_factor, 0, -norm_factor * centroid(0), 0,
            norm_factor, -norm_factor * centroid(1), 0, 0, 1;

        // Apply normalization matrix.
        normed_points->resize(num_points);
        for (size_t i = 0; i < num_points; ++i) {
            (*normed_points)[i] =
                (*normed_from_orig * points[i].homogeneous()).hnormalized();
        }
    }
	bool RANSACFundamental::calMdl(const std::vector<int>& samples, Eigen::Matrix3f& mdl)
	{
        if (m_kps1 == nullptr || m_kps2 == nullptr)
            return false;
        int sz = samples.size();

        // Center and normalize image points for better numerical stability.
        std::vector<Eigen::Vector2d> points1(sz);
        std::vector<Eigen::Vector2d> points2(sz);
        for (int i = 0; i < sz; ++i)
        {
            points1[i] = Eigen::Vector2d(m_kps1->at(m_matches[samples[i]](0)).pt.x, m_kps1->at(m_matches[samples[i]](0)).pt.y);
            points2[i] = Eigen::Vector2d(m_kps2->at(m_matches[samples[i]](1)).pt.x, m_kps2->at(m_matches[samples[i]](1)).pt.y);
        }
        std::vector<Eigen::Vector2d>& normed_points1 = points1;
        std::vector<Eigen::Vector2d>& normed_points2 = points2;

        //std::vector<Eigen::Vector2d> normed_points1;
        //std::vector<Eigen::Vector2d> normed_points2;
        //Eigen::Matrix3d normed_from_orig1;
        //Eigen::Matrix3d normed_from_orig2;
        //CenterAndNormalizeImagePoints(points1, &normed_points1, &normed_from_orig1);
        //CenterAndNormalizeImagePoints(points2, &normed_points2, &normed_from_orig2);

        // Setup homogeneous linear equation as x2' * F * x1 = 0.
        Eigen::Matrix<double, Eigen::Dynamic, 9> A(points1.size(), 9);
        for (size_t i = 0; i < points1.size(); ++i) {
            A.row(i) << normed_points2[i].x() *
                normed_points1[i].transpose().homogeneous(),
                normed_points2[i].y()* normed_points1[i].transpose().homogeneous(),
                normed_points1[i].transpose().homogeneous();
        }

        // Solve for the nullspace of the constraint matrix.
        Eigen::Matrix3d Q;
        if (points1.size() == 8) {
            Eigen::Matrix<double, 9, 9> QQ =
                A.transpose().householderQr().householderQ();
            Q = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
                QQ.col(8).data());
        }
        else {
            Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, 9>> svd(
                A, Eigen::ComputeFullV);
            Q = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
                svd.matrixV().col(8).data());
        }

        // Enforcing the internal constraint that two singular values must non-zero
        // and one must be zero.
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(
            Q, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Vector3d singular_values = svd.singularValues();
        singular_values(2) = 0.0;
        const Eigen::Matrix3d F =
            svd.matrixU() * singular_values.asDiagonal() * svd.matrixV().transpose();

        mdl = F.cast<float>();
        //mdl = (normed_from_orig2.transpose() * F * normed_from_orig1).cast<float>();
		return true;
	}
}