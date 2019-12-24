/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

/*!
 * This file implements constraint unit tests.
 * For more intuitive examples, visit the tutorial.
 * \example ConstraintTest.h
 */

#pragma once

namespace ct {
namespace optcon {
namespace example {

const bool verbose = true;

const size_t state_dim = 12;
const size_t control_dim = 4;


//! A pure state constraint term
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class PureStateConstraint_Example : public ct::optcon::ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef typename ct::core::tpl::TraitSelector<SCALAR>::Trait Trait;
    typedef ct::optcon::ConstraintBase<state_dim, control_dim, SCALAR> Base;
    typedef core::StateVector<STATE_DIM, SCALAR> state_vector_t;
    typedef core::ControlVector<CONTROL_DIM, SCALAR> control_vector_t;
    typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> VectorXs;
    typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic> MatrixXs;

    PureStateConstraint_Example()
    {
        Base::lb_.resize(STATE_DIM);
        Base::ub_.resize(STATE_DIM);
        Base::lb_.setZero();
        Base::ub_.setZero();
    }

    PureStateConstraint_Example(const PureStateConstraint_Example& arg) : Base(arg), A_(arg.A_) {}
    virtual ~PureStateConstraint_Example() {}
    virtual PureStateConstraint_Example<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override
    {
        return new PureStateConstraint_Example(*this);
    }

    virtual size_t getConstraintSize() const override { return STATE_DIM; }
    virtual VectorXs evaluate(const state_vector_t& x, const control_vector_t& u, const SCALAR t) override
    {
        return A_ * x;
    }

    virtual Eigen::Matrix<ct::core::ADCGScalar, Eigen::Dynamic, 1> evaluateCppadCg(
        const core::StateVector<STATE_DIM, ct::core::ADCGScalar>& x,
        const core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar>& u,
        ct::core::ADCGScalar t) override
    {
        return A_.template cast<ct::core::ADCGScalar>() * x;
    }

    virtual MatrixXs jacobianState(const state_vector_t& x, const control_vector_t& u, const SCALAR t) override
    {
        return A_;
    }

    void setA(const Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM>& A) { A_ = A; }

private:
    Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM> A_;
};

//! A state input constraint term
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class StateInputConstraint_Example : public ct::optcon::ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef typename ct::core::tpl::TraitSelector<SCALAR>::Trait Trait;
    typedef ct::optcon::ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR> Base;
    typedef core::StateVector<STATE_DIM, SCALAR> state_vector_t;
    typedef core::ControlVector<CONTROL_DIM, SCALAR> control_vector_t;
    typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> VectorXs;
    typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic> MatrixXs;

    StateInputConstraint_Example()
    {
        Base::lb_.resize(CONTROL_DIM);
        Base::ub_.resize(CONTROL_DIM);
        Base::lb_.setZero();
        Base::ub_.setZero();
    }

    StateInputConstraint_Example(const StateInputConstraint_Example& arg)
        : ct::optcon::ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>(arg), A_(arg.A_), B_(arg.B_)
    {
        Base::lb_.resize(CONTROL_DIM);
        Base::ub_.resize(CONTROL_DIM);
        Base::lb_.setZero();
        Base::ub_.setZero();
    }

    virtual StateInputConstraint_Example<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override
    {
        return new StateInputConstraint_Example(*this);
    }

    virtual ~StateInputConstraint_Example() {}
    virtual size_t getConstraintSize() const override { return CONTROL_DIM; }
    VectorXs evaluate(const state_vector_t& x, const control_vector_t& u, const SCALAR t) override
    {
        return A_ * x + B_ * u;
    }
    virtual Eigen::Matrix<ct::core::ADCGScalar, Eigen::Dynamic, 1> evaluateCppadCg(
        const core::StateVector<STATE_DIM, ct::core::ADCGScalar>& x,
        const core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar>& u,
        ct::core::ADCGScalar t) override
    {
        return (A_.template cast<ct::core::ADCGScalar>() * x + B_.template cast<ct::core::ADCGScalar>() * u);
    }

    virtual MatrixXs jacobianState(const state_vector_t& x, const control_vector_t& u, const SCALAR t) override
    {
        return A_;
    }

    virtual MatrixXs jacobianInput(const state_vector_t& x, const control_vector_t& u, const SCALAR t) override
    {
        return B_;
    }

    void setAB(const Eigen::Matrix<SCALAR, CONTROL_DIM, STATE_DIM>& A,
        const Eigen::Matrix<SCALAR, CONTROL_DIM, CONTROL_DIM> B)
    {
        A_ = A;
        B_ = B;
    }

private:
    Eigen::Matrix<SCALAR, CONTROL_DIM, STATE_DIM> A_;
    Eigen::Matrix<SCALAR, CONTROL_DIM, CONTROL_DIM> B_;
};


TEST(pureStateConstraintTest, pureStateConstraintTest)
{
    std::shared_ptr<ct::optcon::ConstraintContainerAD<state_dim, control_dim>> constraintAD(
        new ct::optcon::ConstraintContainerAD<state_dim, control_dim>());

    std::shared_ptr<ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>> constraintAN(
        new ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>());

    Eigen::Matrix<double, state_dim, state_dim> A;
    A.setRandom();

    std::shared_ptr<PureStateConstraint_Example<state_dim, control_dim>> term1_ad(
        new PureStateConstraint_Example<state_dim, control_dim>());
    term1_ad->setName("term1_ad");
    term1_ad->setA(A);

    std::shared_ptr<PureStateConstraint_Example<state_dim, control_dim, double>> term1_an(
        new PureStateConstraint_Example<state_dim, control_dim, double>());
    term1_an->setName("term1_an");
    term1_an->setA(A);

    constraintAD->addIntermediateConstraint(term1_ad, verbose);
    constraintAD->initialize();
    constraintAN->addIntermediateConstraint(term1_an, verbose);
    constraintAN->initialize();

    std::shared_ptr<ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>> constraintAN_cloned(
        constraintAN->clone());
    std::shared_ptr<ct::optcon::ConstraintContainerAD<state_dim, control_dim>> constraintAD_cloned(
        constraintAD->clone());

    size_t nRuns = 100;

    for (size_t i = 0; i < nRuns; i++)
    {
        /* evaluate constraint */
        Eigen::VectorXd g1_ad, g1_an, g1_ad_cl, g1_an_cl;

        Eigen::Matrix<double, state_dim, 1> state;
        state.setRandom();
        Eigen::Matrix<double, control_dim, 1> input;
        input.setRandom();

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> time_distr(0, 100);
        double time = time_distr(gen);

        constraintAN->setCurrentStateAndControl(state, input, time);
        constraintAD->setCurrentStateAndControl(state, input, time);
        constraintAD_cloned->setCurrentStateAndControl(state, input, time);
        constraintAN_cloned->setCurrentStateAndControl(state, input, time);

        g1_an = constraintAN->evaluateIntermediate();
        g1_ad = constraintAD->evaluateIntermediate();
        g1_an_cl = constraintAN_cloned->evaluateIntermediate();
        g1_ad_cl = constraintAD_cloned->evaluateIntermediate();

        // test if constraint violations are the same
        ASSERT_TRUE(g1_an.isApprox(g1_ad));
        ASSERT_TRUE(g1_an.isApprox(g1_ad_cl));
        ASSERT_TRUE(g1_an.isApprox(g1_an_cl));

        Eigen::MatrixXd F_an, F_ad, F_cloned, F_cloned_an;
        F_an.setZero();
        F_ad.setZero();
        F_cloned.setZero();
        F_cloned_an.setZero();

        F_an = constraintAN->jacobianStateIntermediate();
        F_ad = constraintAD->jacobianStateIntermediate();
        F_cloned_an = constraintAN_cloned->jacobianStateIntermediate();
        F_cloned = constraintAD_cloned->jacobianStateIntermediate();

        // compare jacobians

        ASSERT_TRUE(F_an.isApprox(F_ad));
        ASSERT_TRUE(F_an.isApprox(F_cloned));
        ASSERT_TRUE(F_an.isApprox(F_cloned_an));
    }
}


TEST(stateInputConstraintTest, stateInputConstraintTest)
{
    std::shared_ptr<ct::optcon::ConstraintContainerAD<state_dim, control_dim>> constraintAD(
        new ct::optcon::ConstraintContainerAD<state_dim, control_dim>());
    std::shared_ptr<ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>> constraintAN(
        new ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>());

    Eigen::Matrix<double, control_dim, state_dim> A;
    A.setRandom();
    Eigen::Matrix<double, control_dim, control_dim> B;
    B.setRandom();

    std::shared_ptr<StateInputConstraint_Example<state_dim, control_dim>> term1_ad(
        new StateInputConstraint_Example<state_dim, control_dim>());
    term1_ad->setName("term1_ad");
    term1_ad->setAB(A, B);

    std::shared_ptr<StateInputConstraint_Example<state_dim, control_dim, double>> term1_an(
        new StateInputConstraint_Example<state_dim, control_dim, double>());
    term1_an->setName("term1_an");
    term1_an->setAB(A, B);


    // std::cout << "Adding terms to constraintAD" << std::endl;
    constraintAD->addIntermediateConstraint(term1_ad, verbose);
    constraintAD->initialize();
    constraintAN->addIntermediateConstraint(term1_an, verbose);
    constraintAN->initialize();

    /* evaluate constraint */
    Eigen::VectorXd g1_ad, g1_an;

    Eigen::Matrix<double, state_dim, 1> state;
    state.setRandom();
    Eigen::Matrix<double, control_dim, 1> input;
    input.setRandom();
    double time = 1.0;


    constraintAN->setCurrentStateAndControl(state, input, time);
    constraintAD->setCurrentStateAndControl(state, input, time);


    g1_an = constraintAN->evaluateIntermediate();
    g1_ad = constraintAD->evaluateIntermediate();

    // test if constraint violations are the same
    ASSERT_TRUE(g1_an.isApprox(g1_ad));

    Eigen::MatrixXd C_an, C_ad, C_cloned, C_cloned_an;
    Eigen::MatrixXd D_an, D_ad, D_cloned, D_cloned_an;

    C_an = constraintAN->jacobianStateIntermediate();
    C_ad = constraintAD->jacobianStateIntermediate();
    D_an = constraintAN->jacobianInputIntermediate();
    D_ad = constraintAD->jacobianInputIntermediate();


    std::shared_ptr<ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>> constraintAN_cloned(
        constraintAN->clone());

    std::shared_ptr<ct::optcon::ConstraintContainerAD<state_dim, control_dim>> constraintAD_cloned(
        constraintAD->clone());

    C_cloned_an = constraintAN_cloned->jacobianStateIntermediate();
    C_cloned = constraintAD_cloned->jacobianStateIntermediate();
    D_cloned_an = constraintAN_cloned->jacobianInputIntermediate();
    D_cloned = constraintAD_cloned->jacobianInputIntermediate();

    // compare jacobians
    ASSERT_TRUE(C_an.isApprox(C_ad));
    ASSERT_TRUE(C_an.isApprox(C_cloned));
    ASSERT_TRUE(C_an.isApprox(C_cloned_an));

    ASSERT_TRUE(D_an.isApprox(D_ad));
    ASSERT_TRUE(D_an.isApprox(D_cloned));
    ASSERT_TRUE(D_an.isApprox(D_cloned_an));
}

}  // namespace example
}  // namespace optcon
}  // namespace ct
