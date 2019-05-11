/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

/*!
 * @brief A simple 1d constraint term.
 *
 * This term implements sum of states inequality constraints
 * \f$ d_{lb} \leq x_{0} + x_{1} \leq d_{ub} \f$
 *
 */
class StateSumConstraint : public ct::optcon::ConstraintBase<2, 1>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef typename ct::core::tpl::TraitSelector<double>::Trait Trait;
    typedef typename ct::core::tpl::TraitSelector<ct::core::ADCGScalar>::Trait TraitCG;
    typedef ct::optcon::ConstraintBase<2, 1> Base;
    typedef ct::core::StateVector<2> state_vector_t;
    typedef ct::core::ControlVector<1> control_vector_t;

    typedef Eigen::Matrix<double, 1, 2> Jacobian_state_t;
    typedef Eigen::Matrix<double, 1, 1> Jacobian_control_t;

    //! constructor with constraint boundaries.
    StateSumConstraint(double lb, double ub) : lb_(lb), ub_(ub)
    {
        Base::lb_.resize(1);
        Base::ub_.resize(1);
        Base::lb_.setConstant(lb);
        Base::ub_.setConstant(ub);
    }

    virtual ~StateSumConstraint() {}
    virtual StateSumConstraint* clone() const override { return new StateSumConstraint(lb_, ub_); }
    virtual size_t getConstraintSize() const override { return 1; }
    virtual Eigen::VectorXd evaluate(const state_vector_t& x, const control_vector_t& u, const double t) override
    {
        Eigen::Matrix<double, 1, 1> val;
        val.template segment<1>(0) << x(0) + x(1);
        return val;
    }

    virtual Eigen::Matrix<ct::core::ADCGScalar, Eigen::Dynamic, 1> evaluateCppadCg(
        const ct::core::StateVector<2, ct::core::ADCGScalar>& x,
        const ct::core::ControlVector<1, ct::core::ADCGScalar>& u,
        ct::core::ADCGScalar t) override
    {
        Eigen::Matrix<ct::core::ADCGScalar, 1, 1> val;
        val.template segment<1>(0) << x(0) + x(1);
        return val;
    }

private:
    double lb_;
    double ub_;
};