#include "HomographyEstimator.h"



namespace ImageProcess_LYJ
{
    HomographyDecomposer::HomographyDecomposer()
    {}
    HomographyDecomposer::~HomographyDecomposer()
    {}

    bool HomographyDecomposer::decomposeHomographyMatrix(const cv::Mat & H, 
        const cv::Mat & K1, const cv::Mat & K2, 
        const std::vector<cv::Point2f>&pts1, const std::vector<cv::Point2f>&pts2, 
        Eigen::Matrix3d & R_out, Eigen::Vector3d & t_out)
    {
        if (H.rows != 3 || H.cols != 3 || pts1.empty() || pts1.size() != pts2.size()) {
            std::cerr << "输入参数错误！" << std::endl;
            return false;
        }

        // 1. 归一化H：H = K2 * H_norm * K1^{-1} → H_norm = K2^{-1} * H * K1
        cv::Mat K1_inv = K1.inv(), K2_inv = K2.inv();
        cv::Mat H_norm = K2_inv * H * K1;

        // 2. OpenCV分解单应矩阵（得到4组解）
        std::vector<cv::Mat> Rs, ts, ns; // ns: 平面法向量
        cv::decomposeHomographyMat(H_norm, K1, Rs, ts, ns);

        // 3. 验证解（平面场景/纯旋转）
        cv::Mat P1 = K1 * cv::Mat::eye(3, 4, CV_64F); // 相机1投影矩阵
        for (int i = 0; i < Rs.size(); ++i) {
            // 构建相机2投影矩阵 P2 = K2 [R | t]
            cv::Mat P2(3, 4, CV_64F);
            Rs[i].copyTo(P2.colRange(0, 3));
            ts[i].copyTo(P2.col(3));
            P2 = K2 * P2;

            // 三角化验证深度
            std::vector<cv::Point3d> points3D;
            cv::triangulatePoints(P1, P2, pts1, pts2, points3D);

            int positive_depth = 0;
            for (const auto& p3d : points3D) {
                double w = p3d.z;
                if (w == 0) continue;
                double z = p3d.z / w;
                if (z > 1e-6) positive_depth++;
            }

            if (positive_depth > 0.8 * pts1.size()) {
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



	int RANSACHomography::calErr(const Eigen::Matrix3f& _mdl, const std::vector<int>& _datas, std::vector<float>& _errs, std::vector<bool>& _bInls)
	{
        if (m_kps1 == nullptr || m_kps2 == nullptr)
            return false;
        _errs.assign(_datas.size(), 0);
        _bInls.assign(_datas.size(), false);
        int cnt = 0;

        const float& H_00 = _mdl(0, 0);
        const float& H_01 = _mdl(0, 1);
        const float& H_02 = _mdl(0, 2);
        const float& H_10 = _mdl(1, 0);
        const float& H_11 = _mdl(1, 1);
        const float& H_12 = _mdl(1, 2);
        const float& H_20 = _mdl(2, 0);
        const float& H_21 = _mdl(2, 1);
        const float& H_22 = _mdl(2, 2);

        for (int i = 0; i < _datas.size(); ++i) {
            const float& s_0 = m_kps1->at(m_matches[_datas[i]](0)).pt.x;
            const float& s_1 = m_kps1->at(m_matches[_datas[i]](0)).pt.y;
            const float& d_0 = m_kps2->at(m_matches[_datas[i]](1)).pt.x;
            const float& d_1 = m_kps2->at(m_matches[_datas[i]](1)).pt.y;

            const float pd_0 = H_00 * s_0 + H_01 * s_1 + H_02;
            const float pd_1 = H_10 * s_0 + H_11 * s_1 + H_12;
            const float pd_2 = H_20 * s_0 + H_21 * s_1 + H_22;

            const float inv_pd_2 = 1.0 / pd_2;
            const float dd_0 = d_0 - pd_0 * inv_pd_2;
            const float dd_1 = d_1 - pd_1 * inv_pd_2;

            _errs[i] = dd_0 * dd_0 + dd_1 * dd_1;
            if (_errs[i] > m_errTh*m_errTh)
                continue;
            _bInls[i] = true;
            ++cnt;
        }
        return cnt;
	}
	bool RANSACHomography::calMdl(const std::vector<int>& samples, Eigen::Matrix3f& mdl)
	{
        const size_t num_points = samples.size();
        if (m_kps1 == nullptr || m_kps2 == nullptr)
            return false;

        // Setup constraint matrix.
        Eigen::Matrix<double, Eigen::Dynamic, 9> A(2 * num_points, 9);
        Eigen::Vector3d p1(0, 0, 1);
        Eigen::Vector3d p2(0, 0, 1);
        for (size_t i = 0; i < num_points; ++i) {
            p1(0) = m_kps1->at(m_matches[samples[i]](0)).pt.x;
            p1(1) = m_kps1->at(m_matches[samples[i]](0)).pt.y;
            p2(0) = m_kps2->at(m_matches[samples[i]](1)).pt.x;
            p2(1) = m_kps2->at(m_matches[samples[i]](1)).pt.y;
            A.block<1, 3>(2 * i, 0) = p1.transpose();
            A.block<1, 3>(2 * i, 3).setZero();
            A.block<1, 3>(2 * i, 6) =
                -p2.x() * p1.transpose();
            A.block<1, 3>(2 * i + 1, 0).setZero();
            A.block<1, 3>(2 * i + 1, 3) = p1.transpose();
            A.block<1, 3>(2 * i + 1, 6) =
                -p2.y() * p1.transpose();
        }

        Eigen::Matrix3d H;
        if (num_points == 4) {
            const Eigen::Matrix<double, 9, 1> h = A.block<8, 8>(0, 0)
                .partialPivLu()
                .solve(-A.block<8, 1>(0, 8))
                .homogeneous();
            if (h.hasNaN()) {
                return false;
            }
            H = Eigen::Map<const Eigen::Matrix3d>(h.data()).transpose();
        }
        else {
            // Solve for the nullspace of the constraint matrix.
            Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, 9>> svd(
                A, Eigen::ComputeFullV);
            if (svd.rank() < 8) {
                return false;
            }
            const Eigen::VectorXd nullspace = svd.matrixV().col(8);
            H = Eigen::Map<const Eigen::Matrix3d>(nullspace.data()).transpose();
        }

        if (std::abs(H.determinant()) < 1e-8) {
            return false;
        }

        mdl = H.cast<float>();
		return true;
	}
}