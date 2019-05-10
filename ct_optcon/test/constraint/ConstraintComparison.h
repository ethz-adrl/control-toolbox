/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

/*!
 * This file implements constraint unit tests.
 * For more intuitive examples, visit the tutorial.
 * \example ConstraintComparison.h
 */

#pragma once

namespace ct {
namespace optcon {
namespace example {

const bool verbose = true;

const size_t state_dim = 3;
const size_t control_dim = 3;

//! A simple example with an 1d constraint
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class ConstraintTerm1D : public ct::optcon::ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    const static size_t term_dim = 1;
    typedef typename ct::core::tpl::TraitSelector<SCALAR>::Trait Trait;
    typedef typename ct::core::tpl::TraitSelector<ct::core::ADCGScalar>::Trait TraitCG;
    typedef ct::optcon::ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR> Base;
    typedef core::StateVector<STATE_DIM, SCALAR> state_vector_t;
    typedef core::ControlVector<CONTROL_DIM, SCALAR> control_vector_t;
    typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> VectorXs;
    typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic> MatrixXs;

    ConstraintTerm1D()
    {
        Base::lb_.resize(term_dim);
        Base::ub_.resize(term_dim);
        Base::lb_.setZero();
        Base::ub_.setZero();
    }

    virtual ~ConstraintTerm1D() {}
    virtual ConstraintTerm1D<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override
    {
        return new ConstraintTerm1D<STATE_DIM, CONTROL_DIM, SCALAR>();
    }

    virtual size_t getConstraintSize() const override { return term_dim; }
    virtual VectorXs evaluate(const state_vector_t& x, const control_vector_t& u, const SCALAR t) override
    {
        Eigen::Matrix<SCALAR, term_dim, 1> constr_violation;
        constr_violation.template segment<1>(0) << (u(1) * Trait::cos(x(2)) - u(0) * Trait::sin(x(2)) - u(2));
        return constr_violation;
    }

    virtual Eigen::Matrix<ct::core::ADCGScalar, Eigen::Dynamic, 1> evaluateCppadCg(
        const core::StateVector<STATE_DIM, ct::core::ADCGScalar>& x,
        const core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar>& u,
        ct::core::ADCGScalar t) override
    {
        Eigen::Matrix<ct::core::ADCGScalar, term_dim, 1> constr_violation;
        constr_violation.template segment<1>(0) << (u(1) * TraitCG::cos(x(2)) - u(0) * TraitCG::sin(x(2)) - u(2));
        return constr_violation;
    }

    virtual MatrixXs jacobianState(const state_vector_t& x, const control_vector_t& u, const SCALAR t) override
    {
        Eigen::Matrix<SCALAR, term_dim, STATE_DIM> jac;
        jac.setZero();
        jac << SCALAR(0.0), SCALAR(0.0), -u(1) * sin(x(2)) - u(0) * cos(x(2));
        return jac;
    }

    virtual MatrixXs jacobianInput(const state_vector_t& x, const control_vector_t& u, const SCALAR t) override
    {
        Eigen::Matrix<SCALAR, term_dim, CONTROL_DIM> jac;
        jac.setZero();
        jac << -sin(x(2)), cos(x(2)), SCALAR(-1.0);
        return jac;
    }
};


//! A simple example with a 2d constraint
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class ConstraintTerm2D : public ct::optcon::ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    const static size_t term_dim = 2;
    typedef typename ct::core::tpl::TraitSelector<SCALAR>::Trait Trait;
    typedef typename ct::core::tpl::TraitSelector<ct::core::ADCGScalar>::Trait TraitCG;
    typedef ct::optcon::ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR> Base;
    typedef core::StateVector<STATE_DIM, SCALAR> state_vector_t;
    typedef core::ControlVector<CONTROL_DIM, SCALAR> control_vector_t;
    typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> VectorXs;
    typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic> MatrixXs;

    ConstraintTerm2D()
    {
        Base::lb_.resize(term_dim);
        Base::ub_.resize(term_dim);
        Base::lb_.setZero();
        Base::ub_.setZero();
    }

    virtual ~ConstraintTerm2D() {}
    ConstraintTerm2D<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override
    {
        return new ConstraintTerm2D<STATE_DIM, CONTROL_DIM, SCALAR>();
    }

    virtual size_t getConstraintSize() const override { return term_dim; }
    virtual VectorXs evaluate(const state_vector_t& x, const control_vector_t& u, const SCALAR t) override
    {
        Eigen::Matrix<SCALAR, term_dim, 1> constr_violation;
        constr_violation(0) = (u(1) * Trait::cos(x(2)) - u(0) * Trait::sin(x(2)) - u(2));
        constr_violation(1) = (u(2) * Trait::cos(x(1)) - u(2) * Trait::sin(x(1)) - u(1));
        return constr_violation;
    }

    virtual Eigen::Matrix<ct::core::ADCGScalar, Eigen::Dynamic, 1> evaluateCppadCg(
        const core::StateVector<STATE_DIM, ct::core::ADCGScalar>& x,
        const core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar>& u,
        ct::core::ADCGScalar t) override
    {
        Eigen::Matrix<ct::core::ADCGScalar, term_dim, 1> constr_violation;
        constr_violation(0) = (u(1) * TraitCG::cos(x(2)) - u(0) * TraitCG::sin(x(2)) - u(2));
        constr_violation(1) = (u(2) * TraitCG::cos(x(1)) - u(2) * TraitCG::sin(x(1)) - u(1));
        return constr_violation;
    }

    virtual MatrixXs jacobianState(const state_vector_t& x, const control_vector_t& u, const SCALAR t) override
    {
        Eigen::Matrix<SCALAR, term_dim, STATE_DIM> jac;
        jac.setZero();
        jac.row(0) << 0.0, 0.0, -u(1) * sin(x(2)) - u(0) * cos(x(2));
        jac.row(1) << 0.0, -(u(2)) * sin(x(1)) - u(2) * cos(x(1)), 0.0;

        return jac;
    }

    virtual MatrixXs jacobianInput(const state_vector_t& x, const control_vector_t& u, const SCALAR t) override
    {
        Eigen::Matrix<SCALAR, term_dim, CONTROL_DIM> jac;
        jac.setZero();
        jac.row(0) << -sin(x(2)), cos(x(2)), -1.0;
        jac.row(1) << 0.0, -1.0, cos(x(1)) - sin(x(1));
        return jac;
    }
};


TEST(ConstraintComparison, comparisonAnalyticAD)
{
    std::shared_ptr<ct::optcon::ConstraintContainerAD<state_dim, control_dim>> constraintAD(
        new ct::optcon::ConstraintContainerAD<state_dim, control_dim>());

    std::shared_ptr<ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>> constraintAN(
        new ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>());

    std::shared_ptr<ConstraintTerm1D<state_dim, control_dim>> term1_ad(new ConstraintTerm1D<state_dim, control_dim>());
    term1_ad->setName("term1_ad");
    std::shared_ptr<ConstraintTerm2D<state_dim, control_dim>> term2_ad(new ConstraintTerm2D<state_dim, control_dim>());
    term2_ad->setName("term2_ad");

    std::shared_ptr<ConstraintTerm1D<state_dim, control_dim, double>> term1_an(
        new ConstraintTerm1D<state_dim, control_dim, double>());
    term1_an->setName("term1_an");
    std::shared_ptr<ConstraintTerm2D<state_dim, control_dim, double>> term2_an(
        new ConstraintTerm2D<state_dim, control_dim, double>());
    term2_an->setName("term2_an");


    std::cout << "Adding terms to constraint_analytic" << std::endl;
    constraintAD->addIntermediateConstraint(term1_ad, verbose);
    constraintAD->addIntermediateConstraint(term2_ad, verbose);
    constraintAN->addIntermediateConstraint(term1_an, verbose);
    constraintAN->addIntermediateConstraint(term2_an, verbose);

    constraintAD->initialize();
    constraintAN->initialize();

    /* evaluate constraint */
    Eigen::VectorXd g1_ad, g1_an;

    Eigen::Matrix<double, state_dim, 1> state = Eigen::Matrix<double, state_dim, 1>::Random();
    Eigen::Matrix<double, control_dim, 1> control = Eigen::Matrix<double, control_dim, 1>::Random();
    double time = 0.5;

    constraintAN->setCurrentStateAndControl(state, control, time);
    constraintAD->setCurrentStateAndControl(state, control, time);

    g1_an = constraintAN->evaluateIntermediate();
    g1_ad = constraintAD->evaluateIntermediate();

    std::cout << "g1_an: " << g1_an.transpose() << std::endl;
    std::cout << "g1_ad: " << g1_ad.transpose() << std::endl;

    // test if constraint violations are the same
    ASSERT_TRUE(g1_an.isApprox(g1_ad));

    Eigen::MatrixXd C_an, C_ad, D_an, D_ad;
    C_an = constraintAN->jacobianStateIntermediate();
    D_an = constraintAN->jacobianInputIntermediate();
    C_ad = constraintAD->jacobianStateIntermediate();
    D_ad = constraintAD->jacobianInputIntermediate();

    ASSERT_TRUE(C_an.isApprox(C_ad));

    ASSERT_TRUE(D_an.isApprox(D_ad));

    ASSERT_TRUE(1.0);
}

}  // namespace example
}  // namespace optcon
}  // namespace ct
