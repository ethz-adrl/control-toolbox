/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#ifdef CPPADCG

#include "DynamicsLinearizerADBase.h"

namespace ct {
namespace core {

//! Computes the linearization of a system dynamics function through autodiff
/*!
 * This class takes a function handle representing system dynamics of the form
 * \f$ f(x(t),t,u(t),\dot{x(t)}) \f$ or \f$ f(x[n],n,u[n],x[n+1]) \f$ where the
 * last argument is the result of the evaluation in each case. It then computes
 * the linearization around a given point \f$ x = x_s \f$, \f$ u = u_s \f$.
 *
 * \tparam STATE_DIM dimension of state vector
 * \tparam CONTROL_DIM dimension of control vector
 * \tparam SCALAR scalar type
 * \tparam TIME type of time variable of dynamics
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR, typename TIME>
class DynamicsLinearizerAD : public internal::DynamicsLinearizerADBase<STATE_DIM, CONTROL_DIM, SCALAR, TIME>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef internal::DynamicsLinearizerADBase<STATE_DIM, CONTROL_DIM, SCALAR, TIME> Base;

    typedef typename Base::OUT_SCALAR OUT_SCALAR;  //!< scalar type of resulting linear system

    typedef typename Base::state_vector_t state_vector_t;      //!< state vector type
    typedef typename Base::control_vector_t control_vector_t;  //!< control vector type

    typedef typename Base::state_matrix_t state_matrix_t;                  //!< state Jacobian type (A)
    typedef typename Base::state_control_matrix_t state_control_matrix_t;  //!< control Jacobian type (B)

    typedef typename Base::dynamics_fct_t dynamics_fct_t;  //!< dynamics function signature

    //! default constructor
    /*!
     * @param dyn function handle to system dynamics
     */
    DynamicsLinearizerAD(dynamics_fct_t dyn)
        : Base(dyn), dynamics_fct_(dyn), dFdx_(state_matrix_t::Zero()), dFdu_(state_control_matrix_t::Zero())
    {
    }

    //! copy constructor
    DynamicsLinearizerAD(const DynamicsLinearizerAD& rhs)
        : Base(rhs.dynamics_fct_), dynamics_fct_(rhs.dynamics_fct_), dFdx_(rhs.dFdx_), dFdu_(rhs.dFdu_)
    {
    }

    const state_matrix_t& getDerivativeState(const state_vector_t& x, const control_vector_t& u, const TIME t = TIME(0))
    {
        computeA(x, u);
        return dFdx_;
    }

    const state_control_matrix_t& getDerivativeControl(const state_vector_t& x,
        const control_vector_t& u,
        const TIME t = TIME(0))
    {
        computeB(x, u);
        return dFdu_;
    }

protected:
    //! compute the state Jacobian
    /*!
     * @param x state to linearize around
     * @param u input to linearize around
     */
    void computeA(const state_vector_t& x, const control_vector_t& u)
    {
        Eigen::Matrix<OUT_SCALAR, Eigen::Dynamic, 1> input(STATE_DIM + CONTROL_DIM);
        input << x, u;

        Eigen::Matrix<OUT_SCALAR, Eigen::Dynamic, 1> jac(this->A_entries);

        this->f_.SparseJacobianForward(input, this->sparsityA_.sparsity(), this->sparsityA_.row(),
            this->sparsityA_.col(), jac, this->sparsityA_.workJacobian());

        Eigen::Map<Eigen::Matrix<OUT_SCALAR, STATE_DIM, STATE_DIM>> out(jac.data());

        dFdx_ = out;
    }

    //! compute the input Jacobian
    /*!
     * @param x state to linearize around
     * @param u input to linearize around
     */
    void computeB(const state_vector_t& x, const control_vector_t& u)
    {
        Eigen::Matrix<OUT_SCALAR, Eigen::Dynamic, 1> input(STATE_DIM + CONTROL_DIM);
        input << x, u;

        Eigen::Matrix<OUT_SCALAR, Eigen::Dynamic, 1> jac(this->B_entries);

        this->f_.SparseJacobianForward(input, this->sparsityB_.sparsity(), this->sparsityB_.row(),
            this->sparsityB_.col(), jac, this->sparsityB_.workJacobian());

        Eigen::Map<Eigen::Matrix<OUT_SCALAR, STATE_DIM, CONTROL_DIM>> out(jac.data());

        dFdu_ = out;
    }

protected:
    dynamics_fct_t dynamics_fct_;  //!< function handle to system dynamics

    state_matrix_t dFdx_;          //!< Jacobian wrt state
    state_control_matrix_t dFdu_;  //!< Jacobian wrt input
};

}  // namespace core
}  // namespace ct


#endif