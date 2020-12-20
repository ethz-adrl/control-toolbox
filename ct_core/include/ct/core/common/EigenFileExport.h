/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <string>
#include <fstream>
#include <vector>

#include <Eigen/Dense>

namespace ct::core {

//
// @brief Save Eigen-types or vectors of Eigen-types to text file.
// Currently ordinary Eigen matrices and quaternion types are supported.
//
// Usage examples:
// // writing an eigen matrix
// Eigen::Matrix3d m = Eigen::Matrix3d::Random();
// EigenFileExport::mat_to_file(EigenFileExport::OctaveFormat(), "/tmp/eigen_mat.csv", m, "test mat");
//
// // writing a vector of eigen matrices
// std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> m_vec(3, Eigen::Matrix3d::Random());
// EigenFileExport::mat_to_file(EigenFileExport::OctaveFormat(), "/tmp/eigen_mat_vec.csv", m_vec, "some title.");
//
// // writing a quaternion
// Eigen::Quaterniond q(Eigen::Quaterniond::Identity());
// EigenFileExport::quat_to_file(EigenFileExport::OctaveFormat(), "/tmp/quat.csv", q, "test quat");
//
// // writing a vector of quaternions
// std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> q_vec(
//     5, Eigen::Quaterniond::Identity());
// EigenFileExport::quat_to_file(EigenFileExport::OctaveFormat(), "/tmp/quat_vec.csv", q_vec);
//
class EigenFileExport
{
public:
    // Create CSV formatting option, precision can be custimized if desired.
    static const Eigen::IOFormat CSVFormat(int precision = Eigen::StreamPrecision)
    {
        return Eigen::IOFormat(precision, Eigen::DontAlignCols, ", ", "\n");
    }
    // Create Octave formatting option, precision can be custimized if desired.
    static const Eigen::IOFormat OctaveFormat(int precision = Eigen::StreamPrecision)
    {
        return Eigen::IOFormat(precision, 0, ", ", ";\n", "", "", "[", "]");
    }
    // Create comma init formatting option, precision can be custimized if desired.
    static const Eigen::IOFormat CommaInitFormat(int precision = Eigen::StreamPrecision)
    {
        return Eigen::IOFormat(precision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");
    }
    // Create a clean formatting option, precision can be custimized if desired.
    static const Eigen::IOFormat CleanFormat(int precision = Eigen::StreamPrecision)
    {
        return Eigen::IOFormat(precision, 0, ", ", "\n", "[", "]");
    }
    // create default full-info formatting option, precision can be custimized if desired.
    static const Eigen::IOFormat HeavyFormat(int precision = Eigen::FullPrecision)
    {
        return Eigen::IOFormat(precision, 0, ", ", ";\n", "[", "]", "[", "]");
    }

    //! Save a single matrix to single text file.
    template <typename M>
    static void mat_to_file(const Eigen::IOFormat fmt,
        const std::string& filename,
        const Eigen::MatrixBase<M>& m,
        const std::string& heading = "")
    {
        std::ofstream file(filename.c_str());
        if (!heading.empty())
            file << heading << "\n";
        file << m.format(fmt);

        file.close();
    }

    //! Save a single quaternion to a single text file.
    template <typename M>
    static void quat_to_file(const Eigen::IOFormat fmt,
        const std::string& filename,
        const Eigen::QuaternionBase<M>& q,
        const std::string& heading = "")
    {
        std::ofstream file(filename.c_str());
        if (!heading.empty())
            file << heading << "\n";
        file << q.coeffs().transpose().format(fmt);

        file.close();
    }

    // Save a vector of matrices to a single text file.
    template <typename M, typename Alloc = Eigen::aligned_allocator<M>>
    static void mat_to_file(const Eigen::IOFormat fmt,
        const std::string& filename,
        const std::vector<M, Alloc>& m_vec,
        const std::string& heading = "")
    {
        std::ofstream file(filename.c_str());

        if (!heading.empty())
            file << heading << "\n";

        for (size_t i = 0; i < m_vec.size(); i++)
            file << m_vec[i].format(fmt) << "\n";

        file.close();
    }

    // Save a vector of quaternions to a single text file.
    template <typename Q, typename Alloc = Eigen::aligned_allocator<Eigen::Quaternion<Q>>>
    static void quat_to_file(const Eigen::IOFormat fmt,
        const std::string& filename,
        const std::vector<Eigen::Quaternion<Q>, Alloc>& q_vec,
        const std::string& heading = "")
    {
        std::ofstream file(filename.c_str());

        if (!heading.empty())
            file << heading << "\n";

        for (size_t i = 0; i < q_vec.size(); i++)
            file << q_vec[i].coeffs().transpose().format(fmt) << "\n";

        file.close();
    }
};

}  // namespace ct::core
