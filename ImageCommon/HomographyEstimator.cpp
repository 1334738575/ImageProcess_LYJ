#include "HomographyEstimator.h"



namespace ImageProcess_LYJ
{
	int RANSACHomography::calErr(const Eigen::Matrix3f& _mdl, const std::vector<int>& _datas, std::vector<float>& _errs, std::vector<bool>& _bInls)
	{
        if (m_kps1 == nullptr || m_kps2 == nullptr)
            return false;
        _errs.assign(_datas.size(), 0);
        _bInls.assign(_datas.size(), false);
        int cnt = 0;

        const double& H_00 = _mdl(0, 0);
        const double& H_01 = _mdl(0, 1);
        const double& H_02 = _mdl(0, 2);
        const double& H_10 = _mdl(1, 0);
        const double& H_11 = _mdl(1, 1);
        const double& H_12 = _mdl(1, 2);
        const double& H_20 = _mdl(2, 0);
        const double& H_21 = _mdl(2, 1);
        const double& H_22 = _mdl(2, 2);

        for (int i = 0; i < _datas.size(); ++i) {
            const double s_0 = m_kps1->at(m_matches[_datas[i]](0)).pt.x;
            const double s_1 = m_kps1->at(m_matches[_datas[i]](0)).pt.y;
            const double d_0 = m_kps2->at(m_matches[_datas[i]](1)).pt.x;
            const double d_1 = m_kps2->at(m_matches[_datas[i]](1)).pt.y;

            const double pd_0 = H_00 * s_0 + H_01 * s_1 + H_02;
            const double pd_1 = H_10 * s_0 + H_11 * s_1 + H_12;
            const double pd_2 = H_20 * s_0 + H_21 * s_1 + H_22;

            const double inv_pd_2 = 1.0 / pd_2;
            const double dd_0 = d_0 - pd_0 * inv_pd_2;
            const double dd_1 = d_1 - pd_1 * inv_pd_2;

            _errs[i] = dd_0 * dd_0 + dd_1 * dd_1;
            if (_errs[i] > m_errTh)
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