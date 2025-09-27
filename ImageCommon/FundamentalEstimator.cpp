#include "FundamentalEstimator.h"

namespace ImageProcess_LYJ
{
	bool RANSACFundamental::calErr(const Eigen::Matrix3f& mdl, const int& data, float& err)
	{
        if (m_kps1 == nullptr || m_kps2 == nullptr)
            return false;
        Eigen::Vector3f p1(m_kps1->at(data).pt.x, m_kps1->at(data).pt.y, 1);
        Eigen::Vector3f p2(m_kps2->at(data).pt.x, m_kps2->at(data).pt.y, 1);
        err = std::abs(p2.transpose() * mdl * p1);
        if (err < m_errTh)
            return true;
		return false;
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
	bool RANSACFundamental::calMdl(const std::vector<const int*>& samples, Eigen::Matrix3f& mdl)
	{
        if (m_kps1 == nullptr || m_kps2 == nullptr)
            return false;
        int sz = samples.size();

        // Center and normalize image points for better numerical stability.
        std::vector<Eigen::Vector2d> points1(sz);
        std::vector<Eigen::Vector2d> points2(sz);
        for (int i = 0; i < sz; ++i)
        {
            points1[i] = Eigen::Vector2d(m_kps1->at(*samples[i]).pt.x, m_kps1->at(*samples[i]).pt.y);
            points2[i] = Eigen::Vector2d(m_kps2->at(*samples[i]).pt.x, m_kps2->at(*samples[i]).pt.y);
        }
        std::vector<Eigen::Vector2d> normed_points1;
        std::vector<Eigen::Vector2d> normed_points2;
        Eigen::Matrix3d normed_from_orig1;
        Eigen::Matrix3d normed_from_orig2;
        CenterAndNormalizeImagePoints(points1, &normed_points1, &normed_from_orig1);
        CenterAndNormalizeImagePoints(points2, &normed_points2, &normed_from_orig2);

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

        mdl = (normed_from_orig2.transpose() * F * normed_from_orig1).cast<float>();
		return true;
	}
}