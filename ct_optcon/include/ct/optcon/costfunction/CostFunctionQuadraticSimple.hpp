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
template <typename MANIFOLD, size_t CONTROL_DIM>
class CostFunctionQuadraticSimple : public CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using SCALAR = typename MANIFOLD::Scalar;
    static constexpr size_t STATE_DIM = MANIFOLD::TangentDim;

    typedef core::StateMatrix<STATE_DIM, SCALAR> state_matrix_t;
    typedef core::ControlMatrix<CONTROL_DIM, SCALAR> control_matrix_t;
    typedef core::ControlStateMatrix<STATE_DIM, CONTROL_DIM, SCALAR> control_state_matrix_t;
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
        const MANIFOLD& x_nominal,
        const control_vector_t& u_nominal,
        const MANIFOLD& x_final,
        const state_matrix_t& Q_final);

    virtual ~CostFunctionQuadraticSimple();

    CostFunctionQuadraticSimple(const CostFunctionQuadraticSimple& arg);

    /**
     * Clones the cost function.
     * @return
     */
    CostFunctionQuadraticSimple<MANIFOLD, CONTROL_DIM>* clone() const override;

    virtual void setCurrentStateAndControl(const MANIFOLD& x, const control_vector_t& u, const SCALAR& t) override;

    virtual SCALAR evaluateIntermediate() override;

    virtual typename MANIFOLD::Tangent stateDerivativeIntermediate() override;

    virtual state_matrix_t stateSecondDerivativeIntermediate() override;

    virtual control_vector_t controlDerivativeIntermediate() override;

    virtual control_matrix_t controlSecondDerivativeIntermediate() override;

    virtual control_state_matrix_t stateControlDerivativeIntermediate() override;

    virtual SCALAR evaluateTerminal() override;

    virtual typename MANIFOLD::Tangent stateDerivativeTerminal() override;

    virtual state_matrix_t stateSecondDerivativeTerminal() override;

    virtual void updateReferenceState(const MANIFOLD& x_ref) override;

    virtual void updateFinalState(const MANIFOLD& x_final) override;

protected:
    typename MANIFOLD::Tangent x_deviation_;
    MANIFOLD x_nominal_;
    state_matrix_t Q_;
    Eigen::Matrix<typename MANIFOLD::Scalar, STATE_DIM, STATE_DIM> J_curr_; // TODO: rename
    Eigen::Matrix<typename MANIFOLD::Scalar, STATE_DIM, STATE_DIM> J_ref_; // TODO: rename

    control_vector_t u_deviation_;
    control_vector_t u_nominal_;
    control_matrix_t R_;

    MANIFOLD x_final_;
    state_matrix_t Q_final_;
};

}  // namespace optcon
}  // namespace ct
