/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "CostFunctionQuadratic.hpp"

namespace ct {
namespace optcon {

/*!
 * \ingroup CostFunction
 *
 * \brief A simple quadratic cost function
 *
 * A simple, purely-quadratic cost function of the form
 * \f$ J(x,u,t) = \bar{x}^T Q \bar{x} + \bar{u}^T R \bar{u} + \bar{x}^T_f Q_f \bar{x}^T_f \f$
 * where \f$ \bar{x}, \bar{u} \f$ indicate deviations from a nominal (desired) state and control
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class CostFunctionQuadraticSimple : public CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM> state_matrix_t;
    typedef Eigen::Matrix<SCALAR, CONTROL_DIM, CONTROL_DIM> control_matrix_t;
    typedef Eigen::Matrix<SCALAR, CONTROL_DIM, STATE_DIM> control_state_matrix_t;

    typedef core::StateVector<STATE_DIM, SCALAR> state_vector_t;
    typedef core::ControlVector<CONTROL_DIM, SCALAR> control_vector_t;

    /**
     * Constructs a simple, purely quadratic cost function with all zero elements.
     */
    CostFunctionQuadraticSimple();
    /**
     * Constructs a simple, purely quadratic cost function
     * @param Q intermediate state cost weighting
     * @param R intermediate control cost weighting
     * @param x_nominal nominal (desired) state
     * @param u_nominal nominal (desired) control
     * @param x_final nominal (desired) final state
     * @param Q_final final state cost weighting
     */
    CostFunctionQuadraticSimple(const state_matrix_t& Q,
        const control_matrix_t& R,
        const state_vector_t& x_nominal,
        const control_vector_t& u_nominal,
        const state_vector_t& x_final,
        const state_matrix_t& Q_final);

    virtual ~CostFunctionQuadraticSimple();

    CostFunctionQuadraticSimple(const CostFunctionQuadraticSimple& arg);

    /**
     * Clones the cost function.
     * @return
     */
    CostFunctionQuadraticSimple<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override;

    virtual void setCurrentStateAndControl(const state_vector_t& x,
        const control_vector_t& u,
        const SCALAR& t) override;

    virtual SCALAR evaluateIntermediate() override;

    virtual state_vector_t stateDerivativeIntermediate() override;

    virtual state_matrix_t stateSecondDerivativeIntermediate() override;

    virtual control_vector_t controlDerivativeIntermediate() override;

    virtual control_matrix_t controlSecondDerivativeIntermediate() override;

    virtual control_state_matrix_t stateControlDerivativeIntermediate() override;

    virtual SCALAR evaluateTerminal() override;

    virtual state_vector_t stateDerivativeTerminal() override;

    virtual state_matrix_t stateSecondDerivativeTerminal() override;

    virtual void updateReferenceState(const state_vector_t& x_ref) override;

    virtual void updateFinalState(const state_vector_t& x_final) override;

protected:
    state_vector_t x_deviation_;
    state_vector_t x_nominal_;
    state_matrix_t Q_;

    control_vector_t u_deviation_;
    control_vector_t u_nominal_;
    control_matrix_t R_;

    state_vector_t x_final_;
    state_matrix_t Q_final_;
};

}  // namespace optcon
}  // namespace ct
