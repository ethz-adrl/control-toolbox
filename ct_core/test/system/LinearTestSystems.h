#pragma once

#include <ct/core/core.h>

using namespace ct::core;

const size_t tangent_dim = 3;
const size_t control_dim = 3;

using EuclideanState_t = EuclideanState<tangent_dim>;

// class ContEuclideanLTITestSystem final : public LTISystem<EuclideanState_t, CONTINUOUS_TIME>
// {
// public:
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//     ContEuclideanLTITestSystem(const Eigen::Matrix3d A_test,
//         const Eigen::Matrix3d B_test,
//         const EuclideanState_t& m_ref,
//         const Eigen::Vector3d& u_ref)
//     {
//         this->SetLinearizationPoint(m_ref, u_ref);
//         this->A() = A_test;
//         this->B() = B_test;
//     }
// };

// class DiscrEuclideanLTITestSystem final : public LTISystem<EuclideanState_t, DISCRETE_TIME>
// {
// public:
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//     DiscrEuclideanLTITestSystem(const Eigen::Matrix3d A_test,
//         const Eigen::Matrix3d B_test,
//         const EuclideanState_t& m_ref,
//         const Eigen::Vector3d& u_ref)
//     {
//         this->SetLinearizationPoint(m_ref, u_ref);
//         this->A() = A_test;
//         this->B() = B_test;
//     }
// };


#ifdef CT_USE_MANIF

using ManifoldState_t = ManifoldState<manif::SE2, manif::SE2Tangent>;

class ContSE2LTITestSystem final : public LTISystem<ManifoldState_t, CONTINUOUS_TIME>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ContSE2LTITestSystem(const Eigen::Matrix3d& A,
        const Eigen::Matrix3d& B,
        const ManifoldState_t& m_ref,
        const Eigen::Vector3d& u_ref)
        : LTISystem<ManifoldState_t, CONTINUOUS_TIME>(A, B, m_ref, u_ref)
    {
    }
};

class DiscrSE2LTITestSystem final : public LTISystem<ManifoldState_t, DISCRETE_TIME>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    DiscrSE2LTITestSystem(const Eigen::Matrix3d A,
        const Eigen::Matrix3d B,
        const ManifoldState_t& m_ref,
        const Eigen::Vector3d& u_ref)
        : LTISystem<ManifoldState_t, DISCRETE_TIME>(A, B, m_ref, u_ref)
    {
    }
};

#endif  // CT_USE_MANIF
