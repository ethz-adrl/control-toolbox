/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/
#pragma once

#ifdef CPPAD

namespace ct {
namespace core {

//! Computes the linearization of a general non-linear DiscreteControlledSystem using Automatic Differentiation (without code generation)
/*!
 * This class takes a non-linear DiscreteControlledSystem \f$ x[n+1] = f(x[n],u[n],n) \f$
 * and computes the linearization around a certain point \f$ x = x_s \f$, \f$ u = u_s \f$.
 *
 * \f[
 *   x[n+1] = A x[n] + B u[n]
 * \f]
 *
 * where
 *
 * \f[
 * \begin{aligned}
 * A &= \frac{df}{dx} |_{x=x_s, u=u_s} \\
 * B &= \frac{df}{du} |_{x=x_s, u=u_s}
 * \end{aligned}
 * \f]
 *
 * \note This is generally the most accurate way to generate the linearization of system dynamics together with DiscreteSystemLinearizerADCG.
 * However, the latter is much faster. Consider using the latter for production code.
 *
 * Unit test \ref AutoDiffLinearizerTest.cpp illustrates the use of this class.
 *
 *
 * \warning You should ensure that your DiscreteControlledSystem is templated on the scalar type and does not contain branching
 * (if/else statements, switch cases etc.)
 *
 *
 * \warning This function still has some issues with pure time dependency
 * \todo Make time an Auto-Diff parameter
 *
 * @tparam STATE_DIM dimension of state vector
 * @tparam CONTROL_DIM dimension of control vector
 * @tparam SCALAR primitive type of resultant linear system
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class DiscreteSystemLinearizerAD : public DiscreteLinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef DiscreteLinearSystem<STATE_DIM, CONTROL_DIM, SCALAR> Base;

    typedef CppAD::AD<SCALAR> ADScalar;
    typedef DiscreteControlledSystem<STATE_DIM, CONTROL_DIM, ADScalar> system_t;  //!< type of system to be linearized
    typedef DynamicsLinearizerAD<STATE_DIM, CONTROL_DIM, ADScalar, int>
        linearizer_t;  //!< type of linearizer to be used

    typedef typename Base::state_vector_t state_vector_t;                  //!< state vector type
    typedef typename Base::control_vector_t control_vector_t;              //!< control vector type
    typedef typename Base::state_matrix_t state_matrix_t;                  //!< state Jacobian type (A)
    typedef typename Base::state_control_matrix_t state_control_matrix_t;  //! control Jacobian type (B)

    //! default constructor
    /*!
     * @param nonlinearSystem non-linear system instance to linearize
     */
    DiscreteSystemLinearizerAD(std::shared_ptr<system_t> nonlinearSystem)
        : Base(nonlinearSystem->getType()),
          dFdx_(state_matrix_t::Zero()),
          dFdu_(state_control_matrix_t::Zero()),
          nonlinearSystem_(nonlinearSystem),
          linearizer_(std::bind(&system_t::propagateControlledDynamics,
              nonlinearSystem_.get(),
              std::placeholders::_1,
              std::placeholders::_2,
              std::placeholders::_3,
              std::placeholders::_4))
    {
    }

    //! copy constructor
    DiscreteSystemLinearizerAD(const DiscreteSystemLinearizerAD& arg)
        : Base(arg.nonlinearSystem_->getType()),
          dFdx_(arg.dFdx_),
          dFdu_(arg.dFdu_),
          nonlinearSystem_(arg.nonlinearSystem_->clone()),
          linearizer_(std::bind(&system_t::propagateControlledDynamics,
              nonlinearSystem_.get(),
              std::placeholders::_1,
              std::placeholders::_2,
              std::placeholders::_3,
              std::placeholders::_4))
    {
    }

    //! destructor
    virtual ~DiscreteSystemLinearizerAD() {}
    //! deep cloning
    DiscreteSystemLinearizerAD<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override
    {
        return new DiscreteSystemLinearizerAD<STATE_DIM, CONTROL_DIM, SCALAR>(*this);
    }

    //! get the Jacobian with respect to the state
    /*!
     * This computes the linearization of the system with respect to the state at a given point \f$ x=x_s \f$, \f$ u=u_s \f$,
     * i.e. it computes
     *
     * \f[
     * A = \frac{df}{dx} |_{x=x_s, u=u_s}
     * \f]
     *
     * @param x state to linearize at
     * @param u control to linearize at
     * @param t time
     * @return Jacobian wrt state
     */
    const state_matrix_t& getDerivativeState(const state_vector_t& x, const control_vector_t& u, const int t = 0)
    {
        dFdx_ = linearizer_.getDerivativeState(x, u, t);
        return dFdx_;
    }


    //! get the Jacobian with respect to the input
    /*!
   * This computes the linearization of the system with respect to the input at a given point \f$ x=x_s \f$, \f$ u=u_s \f$,
   * i.e. it computes
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
        const int t = 0)
    {
        dFdu_ = linearizer_.getDerivativeControl(x, u, t);
        return dFdu_;
    }

    //! retrieve discrete-time linear system matrices A and B.
    /*!
     * This computes matrices A and B such that
     * \f[
     *  x_{n+1} = Ax_n + Bu_n
     * \f]
     *
     * Note that the inputs x_next and subSteps are being ignored
     *
     * @param x the state setpoint at n
     * @param u the control setpoint at n
     * @param n the time setpoint
     * @param x_next -> ignored
     * @param subSteps -> ignored
     * @param A the resulting linear system matrix A
     * @param B the resulting linear system matrix B
     */
    void getAandB(const state_vector_t& x,
        const control_vector_t& u,
        const state_vector_t& x_next,
        const int n,
        size_t numSteps,
        state_matrix_t& A,
        state_control_matrix_t& B) override
    {
        dFdx_ = linearizer_.getDerivativeState(x, u, n);
        dFdu_ = linearizer_.getDerivativeControl(x, u, n);

        A = dFdx_;
        B = dFdu_;
    }

protected:
    state_matrix_t dFdx_;
    state_control_matrix_t dFdu_;

    std::shared_ptr<system_t> nonlinearSystem_;  //!< instance of non-linear system

    linearizer_t linearizer_;  //!< instance of ad-linearizer
};

}  // namespace core
}  // namespace ct

#endif