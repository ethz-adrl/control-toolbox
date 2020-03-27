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
template <typename MANIFOLD, typename MANIFOLD_AD, size_t CONTROL_DIM, bool CONT_T>
class DynamicsLinearizerAD : public internal::DynamicsLinearizerADBase<MANIFOLD_AD, CONTROL_DIM, CONT_T>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef internal::DynamicsLinearizerADBase<MANIFOLD_AD, CONTROL_DIM, CONT_T> Base;
    static constexpr size_t STATE_DIM = Base::STATE_DIM;
    using SCALAR = typename MANIFOLD_AD::Scalar;
    using Time_t = typename ControlledSystem<MANIFOLD, CONTROL_DIM, CONT_T>::Time_t;

    typedef typename Base::OUT_SCALAR OUT_SCALAR;  //!< scalar type of resulting linear system
    typedef typename Base::control_vector_t control_vector_t;
    typedef typename Base::state_matrix_t state_matrix_t;
    typedef typename Base::state_control_matrix_t state_control_matrix_t;

    typedef typename Base::dynamics_fct_t dynamics_fct_t;
    typedef typename Base::lift_fct_t lift_fct_t;
    typedef typename Base::retract_fct_t retract_fct_t;

    //! default constructor
    /*!
     * @param dyn function handle to system dynamics
     */
    DynamicsLinearizerAD(dynamics_fct_t dyn, lift_fct_t lift, retract_fct_t retract) : Base(dyn, lift, retract) {}
    //! copy constructor
    DynamicsLinearizerAD(const DynamicsLinearizerAD& rhs) : Base(rhs) {}
    const state_matrix_t& getDerivativeState(const MANIFOLD& x, const control_vector_t& u, const Time_t t = Time_t(0))
    {
        computeA(x, u);
        return this->dFdx_;  // TODO this smells...
    }

    const state_control_matrix_t& getDerivativeControl(const MANIFOLD& x,
        const control_vector_t& u,
        const Time_t t = Time_t(0))
    {
        computeB(x, u);
        return this->dFdu_;  // TODO this smells...
    }

protected:
    //! compute the state Jacobian
    /*!
     * @param x state to linearize around
     * @param u input to linearize around
     */
    void computeA(const MANIFOLD& x, const control_vector_t& u)
    {
        Eigen::Matrix<OUT_SCALAR, Eigen::Dynamic, 1> input(STATE_DIM + CONTROL_DIM);
        input << x, u;

        Eigen::Matrix<OUT_SCALAR, Eigen::Dynamic, 1> jac(this->A_entries);

        this->f_.SparseJacobianForward(input, this->sparsityA_.sparsity(), this->sparsityA_.row(),
            this->sparsityA_.col(), jac, this->sparsityA_.workJacobian());

        Eigen::Map<Eigen::Matrix<OUT_SCALAR, STATE_DIM, STATE_DIM>> out(jac.data());

        this->dFdx_ = out;
    }

    //! compute the input Jacobian
    /*!
     * @param x state to linearize around
     * @param u input to linearize around
     */
    void computeB(const MANIFOLD& x, const control_vector_t& u)
    {
        Eigen::Matrix<OUT_SCALAR, Eigen::Dynamic, 1> input(STATE_DIM + CONTROL_DIM);
        input << x, u;

        Eigen::Matrix<OUT_SCALAR, Eigen::Dynamic, 1> jac(this->B_entries);

        this->f_.SparseJacobianForward(input, this->sparsityB_.sparsity(), this->sparsityB_.row(),
            this->sparsityB_.col(), jac, this->sparsityB_.workJacobian());

        Eigen::Map<Eigen::Matrix<OUT_SCALAR, STATE_DIM, CONTROL_DIM>> out(jac.data());

        this->dFdu_ = out;
    }
};

}  // namespace core
}  // namespace ct

#endif  // CPPADCG