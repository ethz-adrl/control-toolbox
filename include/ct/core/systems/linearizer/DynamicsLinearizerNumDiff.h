/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/
#pragma once

namespace ct {
namespace core {

//! Computes the linearization of a system dynamics function through numerical finite differencing
/*!
 * This class takes a function handle representing system dynamics of the form
 * \f$ f(x(t),t,u(t),\dot{x(t)}) \f$ or \f$ f(x[n],n,u[n],x[n+1]) \f$ where the
 * last argument is the result of the evaluation in each case. It then computes
 * the linearization around a given point \f$ x = x_s \f$, \f$ u = u_s \f$.
 *
 * Finite differencing is used to calculate partial derivatives
 * \f[
 * \begin{aligned}
 * A &= \frac{df}{dx} |_{x=x_s, u=u_s} \\
 * B &= \frac{df}{du} |_{x=x_s, u=u_s}
 * \end{aligned}
 * \f]
 *
 */

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR, typename TIME>
class DynamicsLinearizerNumDiff
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef StateVector<STATE_DIM, SCALAR> state_vector_t;        //!< state vector type
    typedef ControlVector<CONTROL_DIM, SCALAR> control_vector_t;  //!< control vector type

    typedef StateMatrix<STATE_DIM, SCALAR> state_matrix_t;                              //!< state Jacobian type (A)
    typedef StateControlMatrix<STATE_DIM, CONTROL_DIM, SCALAR> state_control_matrix_t;  //!< control Jacobian type (B)

    typedef std::function<void(const state_vector_t&, const TIME&, const control_vector_t&, state_vector_t&)>
        dynamics_fct_t;  //!< dynamics function signature

    //! default constructor
    /*!
     * Initializes the linearizer with a dynamics function object
     *
     * @param nonlinearSystem non-linear system to linearize
     * @param doubleSidedDerivative if true, double sided numerical differentiation is used
     */
    DynamicsLinearizerNumDiff(dynamics_fct_t dyn, bool doubleSidedDerivative = true)
        : dynamics_fct_(dyn), doubleSidedDerivative_(doubleSidedDerivative)
    {
        dFdx_.setZero();
        dFdu_.setZero();
        eps_ = sqrt(Eigen::NumTraits<SCALAR>::epsilon());
    }

    //! copy constructor
    DynamicsLinearizerNumDiff(const DynamicsLinearizerNumDiff& rhs)
        : dynamics_fct_(rhs.dynamics_fct_),
          doubleSidedDerivative_(rhs.doubleSidedDerivative_),
          dFdx_(rhs.dFdx_),
          dFdu_(rhs.dFdu_),
          eps_(rhs.eps_)
    {
    }


    //! get the Jacobian with respect to the state
    /*!
   * This computes the linearization of the dynamics with respect to the state
   * at a given point \f$ x=x_s \f$, \f$ u=u_s \f$, i.e. it computes
   *
   * \f[
   * B = \frac{df}{dx} |_{x=x_s, u=u_s}
   * \f]
   *
   * @param x state to linearize at
   * @param u control to linearize at
   * @param t time
   * @return Jacobian wrt state
   */
    const state_matrix_t& getDerivativeState(const state_vector_t& x, const control_vector_t& u, const TIME t = TIME(0))
    {
        if (!doubleSidedDerivative_)
            dynamics_fct_(x, t, u, res_ref_);

        for (size_t i = 0; i < STATE_DIM; i++)
        {
            // inspired from http://en.wikipedia.org/wiki/Numerical_differentiation#Practical_considerations_using_floating_point_arithmetic
            SCALAR h = eps_ * std::max(std::abs<SCALAR>(x(i)), SCALAR(1.0));
            SCALAR x_ph = x(i) + h;
            SCALAR dxp = x_ph - x(i);

            state_vector_t x_perturbed = x;
            x_perturbed(i) = x_ph;

            // evaluate dynamics at perturbed state
            state_vector_t res_plus;
            dynamics_fct_(x_perturbed, t, u, res_plus);

            if (doubleSidedDerivative_)
            {
                SCALAR x_mh = x(i) - h;
                SCALAR dxm = x(i) - x_mh;

                x_perturbed = x;
                x_perturbed(i) = x_mh;

                state_vector_t res_minus;
                dynamics_fct_(x_perturbed, t, u, res_minus);

                dFdx_.col(i) = (res_plus - res_minus) / (dxp + dxm);
            }
            else
            {
                dFdx_.col(i) = (res_plus - res_ref_) / dxp;
            }
        }

        return dFdx_;
    }

    //! get the Jacobian with respect to the input
    /*!
   * This computes the linearization of the dynamics with respect to the input
   * at a given point \f$ x=x_s \f$, \f$ u=u_s \f$, i.e. it computes
   *
   * \f[
   * B = \frac{df}{du} |_{x=x_s, u=u_s}
   * \f]
   *
   * @param x state to linearize at
   * @param u control to linearize at
   * @param t time
   * @return Jacobian wrt input
   */
    const state_control_matrix_t& getDerivativeControl(const state_vector_t& x,
        const control_vector_t& u,
        const TIME t = TIME(0))
    {
        if (!doubleSidedDerivative_)
            dynamics_fct_(x, t, u, res_ref_);

        for (size_t i = 0; i < CONTROL_DIM; i++)
        {
            // inspired from http://en.wikipedia.org/wiki/Numerical_differentiation#Practical_considerations_using_floating_point_arithmetic
            SCALAR h = eps_ * std::max(std::abs<SCALAR>(u(i)), SCALAR(1.0));
            SCALAR u_ph = u(i) + h;
            SCALAR dup = u_ph - u(i);

            control_vector_t u_perturbed = u;
            u_perturbed(i) = u_ph;

            // evaluate dynamics at perturbed state
            state_vector_t res_plus;
            dynamics_fct_(x, t, u_perturbed, res_plus);

            if (doubleSidedDerivative_)
            {
                SCALAR u_mh = u(i) - h;
                SCALAR dum = u(i) - u_mh;

                u_perturbed = u;
                u_perturbed(i) = u_mh;

                state_vector_t res_minus;
                dynamics_fct_(x, t, u_perturbed, res_minus);

                dFdu_.col(i) = (res_plus - res_minus) / (dup + dum);
            }
            else
            {
                dFdu_.col(i) = (res_plus - res_ref_) / dup;
            }
        }

        return dFdu_;
    }

    bool getDoubleSidedDerivativeFlag() const { return doubleSidedDerivative_; }
protected:
    dynamics_fct_t dynamics_fct_;  //!< function handle to system dynamics

    bool doubleSidedDerivative_;  //!< flag if double sided numerical differentiation should be used

    SCALAR eps_;  //!< perturbation for numerical differentiation


    // internally used variables
    state_matrix_t dFdx_;          //!< Jacobian wrt state
    state_control_matrix_t dFdu_;  //!< Jacobian wrt input

    state_vector_t res_ref_;  //!< reference result for numerical differentiation
};

}  // namespace core
}  // namespace ct
