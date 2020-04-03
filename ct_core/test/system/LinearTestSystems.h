#pragma once

#include <ct/core/core.h>

using namespace ct::core;

const size_t tangent_dim = 3;
const size_t control_dim = 3;

using EuclideanState_t = EuclideanState<tangent_dim>;

class ContEuclideanLTITestSystem final : public LTISystem<EuclideanState_t, control_dim, CONTINUOUS_TIME>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ContEuclideanLTITestSystem(const Eigen::Matrix3d A_test,
        const Eigen::Matrix3d B_test,
        const EuclideanState_t& m_ref)
        : m_ref_(m_ref)
    {
        this->A() = A_test;
        this->B() = B_test;
    }

    //! substract the reference point m_ref_ at lifting
    virtual EuclideanState_t::Tangent lift(const EuclideanState_t& m) override { return m - m_ref_; }
    virtual EuclideanState_t retract(const EuclideanState_t::Tangent& t) override { return m_ref_ + t; }
protected:
    EuclideanState_t m_ref_;
};

class DiscrEuclideanLTITestSystem final : public LTISystem<EuclideanState_t, control_dim, DISCRETE_TIME>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    DiscrEuclideanLTITestSystem(const Eigen::Matrix3d A_test,
        const Eigen::Matrix3d B_test,
        const EuclideanState_t& m_ref)
        : m_ref_(m_ref)
    {
        this->A() = A_test;
        this->B() = B_test;
    }

    //! substract the reference point m_ref_ at lifting
    virtual EuclideanState_t::Tangent lift(const EuclideanState_t& m) override { return m - m_ref_; }
    virtual EuclideanState_t retract(const EuclideanState_t::Tangent& t) override { return m_ref_ + t; }
protected:
    EuclideanState_t m_ref_;
};


#ifdef CT_USE_MANIF

using ManifoldState_t = ManifoldState<manif::SE2, manif::SE2Tangent>;

class ContSE2LTITestSystem final : public LTISystem<ManifoldState_t, control_dim, CONTINUOUS_TIME>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ContSE2LTITestSystem(const Eigen::Matrix3d A_test, const Eigen::Matrix3d B_test, const ManifoldState_t& m_ref)
        : m_ref_(m_ref)
    {
        this->A() << A_test;
        this->B() << B_test;
    }

    /**
     * @brief the log operator is defined as expressing the tangent vector w.r.t. m_ref_
     */
    virtual manif::SE2Tangentd lift(const ManifoldState_t& m) override { return m.rminus(m_ref_); }
    virtual ManifoldState_t retract(const ManifoldState_t::Tangent& t) override { return m_ref_.rplus(t); }
protected:
    ManifoldState_t m_ref_;
};

class DiscrSE2LTITestSystem final : public LTISystem<ManifoldState_t, control_dim, DISCRETE_TIME>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    DiscrSE2LTITestSystem(const Eigen::Matrix3d A_test, const Eigen::Matrix3d B_test, const ManifoldState_t& m_ref)
        : m_ref_(m_ref)
    {
        this->A() << A_test;
        this->B() << B_test;
    }

    /**
     * @brief the log operator is defined as expressing the tangent vector w.r.t. m_ref_
     */
    virtual manif::SE2Tangentd lift(const ManifoldState_t& m) override { return m.rminus(m_ref_); }
    virtual ManifoldState_t retract(const ManifoldState_t::Tangent& t) override { return m_ref_.rplus(t); }
protected:
    ManifoldState_t m_ref_;
};

#endif  // CT_USE_MANIF
