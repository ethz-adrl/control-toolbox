/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/
#pragma once

namespace ct {
namespace core {

//! Computes the linearization of a general non-linear DiscreteControlledSystem using numerical differentiation
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
 * Examples for using the SystemLinearizer (and the Auto-diff Linearizer) can be found in \ref AutoDiffLinearizerTest.cpp
 *
 * \note In case your ControlledSystem is templated on scalar type, we suggest using the DiscreteSystemLinearizerAD
 * for highest efficiency and accuracy.
 *
 * @tparam dimension of state vector
 * @tparam dimension of control vector
 * @tparam SCALAR underlying scalar type of the system
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class DiscreteSystemLinearizer : public DiscreteLinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef DiscreteLinearSystem<STATE_DIM, CONTROL_DIM, SCALAR> Base;

    typedef DiscreteControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR> system_t;  //!< type of system to be linearized

    typedef typename Base::state_vector_t state_vector_t;      //!< state vector type
    typedef typename Base::control_vector_t control_vector_t;  //!< control vector type

    typedef typename Base::state_matrix_t state_matrix_t;                  //!< state Jacobian type (A)
    typedef typename Base::state_control_matrix_t state_control_matrix_t;  //! control Jacobian type (B)

    //! default constructor
    /*!
     * Initializes the linearizer with a non-linear system.
     *
     * @param nonlinearSystem non-linear system to linearize
     * @param doubleSidedDerivative if true, double sided numerical differentiation is used
     */
    DiscreteSystemLinearizer(std::shared_ptr<system_t> nonlinearSystem, bool doubleSidedDerivative = true)
        : Base(nonlinearSystem->getType()),
          nonlinearSystem_(nonlinearSystem),
          linearizer_(std::bind(&system_t::propagateControlledDynamics,
                          nonlinearSystem_.get(),
                          std::placeholders::_1,
                          std::placeholders::_2,
                          std::placeholders::_3,
                          std::placeholders::_4),
              doubleSidedDerivative)
    {
        if (nonlinearSystem == nullptr)
            throw std::runtime_error("SystemLinearizer: Nonlinear system is nullptr!");

        dFdx_.setZero();
        dFdu_.setZero();
    }

    //! copy constructor
    DiscreteSystemLinearizer(const DiscreteSystemLinearizer& arg)
        : Base(arg),
          nonlinearSystem_(arg.nonlinearSystem_->clone()),
          linearizer_(std::bind(&system_t::propagateControlledDynamics,
                          nonlinearSystem_.get(),
                          std::placeholders::_1,
                          std::placeholders::_2,
                          std::placeholders::_3,
                          std::placeholders::_4),
              arg.linearizer_.getDoubleSidedDerivativeFlag()),
          dFdx_(arg.dFdx_),
          dFdu_(arg.dFdu_)
    {
    }

    //! destructor
    virtual ~DiscreteSystemLinearizer() {}
    //! deep cloning
    DiscreteSystemLinearizer<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override
    {
        return new DiscreteSystemLinearizer<STATE_DIM, CONTROL_DIM, SCALAR>(*this);
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
     * @param n index
     * @return Jacobian wrt state
     */
    virtual const state_matrix_t& getDerivativeState(const state_vector_t& x,
        const control_vector_t& u,
        const int n = 0)
    {
        dFdx_ = linearizer_.getDerivativeState(x, u, n);
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
     * @param n index
     * @return Jacobian wrt input
     */
    virtual const state_control_matrix_t& getDerivativeControl(const state_vector_t& x,
        const control_vector_t& u,
        const int n = 0)
    {
        dFdu_ = linearizer_.getDerivativeControl(x, u, n);
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
    virtual void getAandB(const state_vector_t& x,
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
    std::shared_ptr<system_t> nonlinearSystem_;  //!< instance of non-linear system

    DynamicsLinearizerNumDiff<STATE_DIM, CONTROL_DIM, SCALAR, int> linearizer_;  //!< instance of numerical-linearizer

    state_matrix_t dFdx_;          //!< Jacobian wrt state
    state_control_matrix_t dFdu_;  //!< Jacobian wrt input
};

}  // namespace core
}  // namespace ct
