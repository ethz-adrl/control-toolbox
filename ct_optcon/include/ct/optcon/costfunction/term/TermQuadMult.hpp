/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "TermBase.hpp"

namespace ct {
namespace optcon {

/**
 * \ingroup CostFunction
 *
 * \brief A multiplicative term of type \f$ J = (x^T Q x) (u^T R u) \f$
 *
 * Probably this term is not very useful but we use it for testing.
 *
 * An example for using this term is given in \ref CostFunctionTest.cpp
 *
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL = double, typename SCALAR = SCALAR_EVAL>
class TermQuadMult : public TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Eigen::Matrix<SCALAR_EVAL, STATE_DIM, STATE_DIM> state_matrix_t;
    typedef Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, CONTROL_DIM> control_matrix_t;
    typedef Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, STATE_DIM> control_state_matrix_t;
    typedef Eigen::Matrix<SCALAR_EVAL, STATE_DIM, STATE_DIM> state_matrix_double_t;
    typedef Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, CONTROL_DIM> control_matrix_double_t;
    typedef Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, STATE_DIM> control_state_matrix_double_t;

    TermQuadMult();

    TermQuadMult(const state_matrix_t& Q, const control_matrix_t& R);

    TermQuadMult(const state_matrix_t& Q,
        const control_matrix_t& R,
        const core::StateVector<STATE_DIM, SCALAR_EVAL>& x_ref,
        core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u_ref);

    TermQuadMult(const TermQuadMult& arg);

    virtual ~TermQuadMult();

    TermQuadMult<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>* clone() const override;

    void setWeights(const state_matrix_double_t& Q, const control_matrix_double_t& R);

    void setStateAndControlReference(const core::StateVector<STATE_DIM>& x_ref,
        core::ControlVector<CONTROL_DIM>& u_ref);

    virtual SCALAR evaluate(const Eigen::Matrix<SCALAR, STATE_DIM, 1>& x,
        const Eigen::Matrix<SCALAR, CONTROL_DIM, 1>& u,
        const SCALAR& t) override;

#ifdef CPPADCG
    virtual ct::core::ADCGScalar evaluateCppadCg(const core::StateVector<STATE_DIM, ct::core::ADCGScalar>& x,
        const core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar>& u,
        ct::core::ADCGScalar t) override;
#endif

    core::StateVector<STATE_DIM, SCALAR_EVAL> stateDerivative(const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
        const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
        const SCALAR_EVAL& t) override;

    state_matrix_t stateSecondDerivative(const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
        const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
        const SCALAR_EVAL& t) override;

    core::ControlVector<CONTROL_DIM, SCALAR_EVAL> controlDerivative(const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
        const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
        const SCALAR_EVAL& t) override;

    control_matrix_t controlSecondDerivative(const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
        const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
        const SCALAR_EVAL& t) override;

    control_state_matrix_t stateControlDerivative(const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
        const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
        const SCALAR_EVAL& t) override;

    void loadConfigFile(const std::string& filename, const std::string& termName, bool verbose = false) override;


protected:
    template <typename SC>
    SC evalLocal(const Eigen::Matrix<SC, STATE_DIM, 1>& x, const Eigen::Matrix<SC, CONTROL_DIM, 1>& u, const SC& t);

    state_matrix_t Q_;
    control_matrix_t R_;

    core::StateVector<STATE_DIM, SCALAR_EVAL> x_ref_;
    core::ControlVector<CONTROL_DIM, SCALAR_EVAL> u_ref_;
};


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
template <typename SC>
SC TermQuadMult<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::evalLocal(const Eigen::Matrix<SC, STATE_DIM, 1>& x,
    const Eigen::Matrix<SC, CONTROL_DIM, 1>& u,
    const SC& t)
{
    Eigen::Matrix<SC, STATE_DIM, 1> xDiff = (x - x_ref_.template cast<SC>());
    Eigen::Matrix<SC, CONTROL_DIM, 1> uDiff = (u - u_ref_.template cast<SC>());

    return (xDiff.transpose() * Q_.template cast<SC>() * xDiff)(0, 0) *
           (uDiff.transpose() * R_.template cast<SC>() * uDiff)(0, 0);
}

}  // namespace optcon
}  // namespace ct
