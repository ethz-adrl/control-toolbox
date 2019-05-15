/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#define EIGEN_INITIALIZE_MATRICES_BY_NAN

namespace ct {
namespace optcon {
namespace example {

/*
 * Define a cost function term of form cost = c*t^2 (x'Qx + u'Ru)
 * analytical derivatives:
 * state Derivative: 	2*c*t^2 Qx
 * control Derivative: 	2*c*t^2 Rx
 * state second Derivative: 	2*c*t^2 Q
 * control second Derivative: 	2*c*t^2 R
 * */

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL = double, typename SCALAR = SCALAR_EVAL>
class TestTerm : public TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Eigen::Matrix<SCALAR_EVAL, STATE_DIM, STATE_DIM> state_matrix_t;
    typedef Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, CONTROL_DIM> control_matrix_t;
    typedef Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, STATE_DIM> control_state_matrix_t;
    typedef Eigen::Matrix<SCALAR_EVAL, STATE_DIM, STATE_DIM> state_matrix_double_t;
    typedef Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, CONTROL_DIM> control_matrix_double_t;
    typedef Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, STATE_DIM> control_state_matrix_double_t;


    TestTerm();

    TestTerm(const state_matrix_t& Q, const control_matrix_t& R, const SCALAR_EVAL& c) : Q_(Q), R_(R), c_(c)
    {
        x_ref_.setZero();  // default values
        u_ref_.setZero();
    }

    TestTerm(const TestTerm& arg)
        : TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>(arg),
          Q_(arg.Q_),
          R_(arg.R_),
          c_(arg.c_),
          x_ref_(arg.x_ref_),
          u_ref_(arg.u_ref_)
    {
    }

    virtual ~TestTerm() {}
    TestTerm<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>* clone() const override { return new TestTerm(*this); }
    void setWeights(const state_matrix_t& Q, const control_matrix_t& R, const double& c)
    {
        Q_ = Q;
        R_ = R;
        c_ = c;
    }

    void setStateAndControlReference(const core::StateVector<STATE_DIM, SCALAR_EVAL>& x_ref,
        core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u_ref)
    {
        x_ref_ = x_ref;
        u_ref_ = u_ref;
    }

    virtual SCALAR evaluate(const Eigen::Matrix<SCALAR, STATE_DIM, 1>& x,
        const Eigen::Matrix<SCALAR, CONTROL_DIM, 1>& u,
        const SCALAR& t) override
    {
        return evalLocal<SCALAR>(x, u, t);
    }

    virtual ct::core::ADCGScalar evaluateCppadCg(const core::StateVector<STATE_DIM, ct::core::ADCGScalar>& x,
        const core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar>& u,
        ct::core::ADCGScalar t) override
    {
        return evalLocal<ct::core::ADCGScalar>(x, u, t);
    }


    // todo: this is quite ugly
    core::StateVector<STATE_DIM, SCALAR_EVAL> stateDerivativeAnalytical(const core::StateVector<STATE_DIM>& x,
        const core::ControlVector<CONTROL_DIM>& u,
        const double t)
    {
        return (2 * c_ * (SCALAR_EVAL)t * (SCALAR_EVAL)t * Q_ * x.template cast<SCALAR_EVAL>());
    }

    // todo: this is very ugly
    core::ControlVector<CONTROL_DIM, SCALAR_EVAL> controlDerivativeAnalytical(const core::StateVector<STATE_DIM>& x,
        const core::ControlVector<CONTROL_DIM>& u,
        double t)
    {
        return (2 * c_ * (SCALAR_EVAL)t * (SCALAR_EVAL)t * R_ * u.template cast<SCALAR_EVAL>());
    }

    // todo: this is quite ugly
    state_matrix_t stateSecondDerivativeAnalytical(const core::StateVector<STATE_DIM>& x,
        const core::ControlVector<CONTROL_DIM>& u,
        const double t)
    {
        return (2 * c_ * (SCALAR_EVAL)t * (SCALAR_EVAL)t * Q_);
    }

    // todo: this is very ugly
    control_matrix_t controlSecondDerivativeAnalytical(const core::StateVector<STATE_DIM>& x,
        const core::ControlVector<CONTROL_DIM>& u,
        double t)
    {
        return (2 * c_ * (SCALAR_EVAL)t * (SCALAR_EVAL)t * R_);
    }


protected:
    template <typename SC>
    SC evalLocal(const Eigen::Matrix<SC, STATE_DIM, 1>& x, const Eigen::Matrix<SC, CONTROL_DIM, 1>& u, const SC& t)
    {
        Eigen::Matrix<SC, STATE_DIM, 1> xDiff = (x - x_ref_.template cast<SC>());
        Eigen::Matrix<SC, CONTROL_DIM, 1> uDiff = (u - u_ref_.template cast<SC>());

        return ((xDiff.transpose() * Q_.template cast<SC>() * xDiff) * SC(c_) * t * t +
                (uDiff.transpose() * R_.template cast<SC>() * uDiff) * SC(c_) * t * t)(0, 0);
    }

    state_matrix_t Q_;
    control_matrix_t R_;
    SCALAR_EVAL c_;

    core::StateVector<STATE_DIM, SCALAR_EVAL> x_ref_;
    core::ControlVector<CONTROL_DIM, SCALAR_EVAL> u_ref_;
};


TEST(AD_TEST_TIME_VAR, AD_TEST_TIME_VAR)
{
    using namespace ct;

    Eigen::Matrix<double, 3, 3> Q;
    Q.setIdentity();
    Eigen::Matrix<double, 3, 3> R;
    R.setIdentity();

    Eigen::Matrix<double, 3, 3> Q_f = 10 * Q;
    Eigen::Matrix<double, 3, 3> R_f = 10 * R;

    double c = 1.0;
    double c_f = 2.0;

    std::shared_ptr<TestTerm<3, 3, double, ct::core::ADCGScalar>> term_intermediate(
        new TestTerm<3, 3, double, ct::core::ADCGScalar>(Q, R, c));
    std::shared_ptr<TestTerm<3, 3, double, ct::core::ADCGScalar>> term_final(
        new TestTerm<3, 3, double, ct::core::ADCGScalar>(Q_f, R_f, c_f));

    // autodiff costfunction
    std::shared_ptr<ct::optcon::CostFunctionAD<3, 3>> ADcf(new ct::optcon::CostFunctionAD<3, 3>());
    ADcf->addIntermediateADTerm(term_intermediate);
    ADcf->addFinalADTerm(term_final);


    Eigen::Vector3d x;
    x.setRandom();
    Eigen::Vector3d u;
    u.setRandom();

    double t_final = 4.0;

    ADcf->initialize();

    for (double t = 0.0; t <= t_final; t = t + 1)
    {
        ADcf->setCurrentStateAndControl(x, u, t);

        Eigen::Matrix<double, 3, 1> diff1 = (ADcf->stateDerivativeTerminal()).template cast<double>() -
                                            (term_final->stateDerivativeAnalytical(x, u, t));
        ASSERT_TRUE(diff1.maxCoeff() - diff1.minCoeff() < 1e-9);

        Eigen::Matrix<double, 3, 1> diff2 = (ADcf->stateDerivativeIntermediate()).template cast<double>() -
                                            (term_intermediate->stateDerivativeAnalytical(x, u, t));
        ASSERT_TRUE(diff2.maxCoeff() - diff2.minCoeff() < 1e-9);

        Eigen::Matrix<double, 3, 1> diff3 = (ADcf->controlDerivativeIntermediate()).template cast<double>() -
                                            (term_intermediate->controlDerivativeAnalytical(x, u, t));
        ASSERT_TRUE(diff3.maxCoeff() - diff3.minCoeff() < 1e-9);

        Eigen::Matrix<double, 3, 1> diff4 = (ADcf->controlDerivativeTerminal()).template cast<double>() -
                                            (term_final->controlDerivativeAnalytical(x, u, t));
        ASSERT_TRUE(diff4.maxCoeff() - diff4.minCoeff() < 1e-9);

        Eigen::Matrix<double, 3, 3> diff5 = (ADcf->controlSecondDerivativeIntermediate()).template cast<double>() -
                                            (term_intermediate->controlSecondDerivativeAnalytical(x, u, t));
        ASSERT_TRUE(diff5.maxCoeff() - diff5.minCoeff() < 1e-9);

        Eigen::Matrix<double, 3, 3> diff6 = (ADcf->controlSecondDerivativeTerminal()).template cast<double>() -
                                            (term_final->controlSecondDerivativeAnalytical(x, u, t));
        ASSERT_TRUE(diff6.maxCoeff() - diff6.minCoeff() < 1e-9);

        Eigen::Matrix<double, 3, 3> diff7 = (ADcf->stateSecondDerivativeIntermediate()).template cast<double>() -
                                            (term_intermediate->stateSecondDerivativeAnalytical(x, u, t));
        ASSERT_TRUE(diff7.maxCoeff() - diff7.minCoeff() < 1e-9);

        Eigen::Matrix<double, 3, 3> diff8 = (ADcf->stateSecondDerivativeTerminal()).template cast<double>() -
                                            (term_final->stateSecondDerivativeAnalytical(x, u, t));
        ASSERT_TRUE(diff8.maxCoeff() - diff8.minCoeff() < 1e-9);
    }
}

}  // namespace example
}  // namespace optcon
}  // namespace ct
