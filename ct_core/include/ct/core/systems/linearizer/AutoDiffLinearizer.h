/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#ifdef CPPAD

namespace ct {
namespace core {

//! Computes the linearization of a general non-linear ControlledSystem using Automatic Differentiation (without code generation)
/*!
 * This class takes a non-linear ControlledSystem \f$ \dot{x} = f(x,u,t) \f$ and computes the linearization
 * around a certain point \f$ x = x_s \f$, \f$ u = u_s \f$.
 *
 * \f[
 *   \dot{x} = A x + B u
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
 * \note This is generally the most accurate way to generate the linearization of system dynamics together with ADCodegenLinearizer.
 * However, the latter is much faster. Consider using the latter for production code.
 *
 * Unit test \ref AutoDiffLinearizerTest.cpp illustrates the use of the AutoDiffLinearizer.
 *
 *
 * \warning You should ensure that your ControlledSystem is templated on the scalar type and does not contain branching
 * (if/else statements, switch cases etc.)
 *
 *
 * \warning This function still has some issues with pure time dependency
 * \todo Make time an Auto-Diff parameter
 *
 * @tparam dimension of state vector
 * @tparam dimension of control vector
 * @tparam SCALAR primitive type of resultant linear system
 */
template <typename MANIFOLD, typename MANIFOLD_AD, size_t CONTROL_DIM, bool CONT_T>
class AutoDiffLinearizer : public LinearSystem<MANIFOLD, CONTROL_DIM, CONT_T>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using SCALAR = typename MANIFOLD::Scalar;

    typedef LinearSystem<MANIFOLD, CONTROL_DIM, CONT_T> Base;  //!< Base class type

    //typedef ControlledSystem<MANIFOLD, CONTROL_DIM, CONT_T> System_t;
    typedef ControlledSystem<MANIFOLD_AD, CONTROL_DIM, CONT_T> AD_System_t;  //!< type of system to be linearized
    typedef DynamicsLinearizerAD<MANIFOLD, MANIFOLD_AD, CONTROL_DIM, CONT_T>
        linearizer_t;  //!< type of linearizer to be used

    typedef typename Base::Time_t Time_t;
    typedef typename Base::control_vector_t control_vector_t;              //!< input vector type
    typedef typename Base::state_matrix_t state_matrix_t;                  //!< state Jacobian type
    typedef typename Base::state_control_matrix_t state_control_matrix_t;  //!< input Jacobian type

    //! default constructor
    /*!
     * @param nonlinearSystem non-linear system instance to linearize
     */
    AutoDiffLinearizer(std::shared_ptr<AD_System_t> nonlinearSystem)
        : Base(nonlinearSystem->getType()),
          dFdx_(state_matrix_t::Zero()),
          dFdu_(state_control_matrix_t::Zero()),
          nonlinearSystem_(nonlinearSystem),
          linearizer_(std::bind(&AD_System_t::computeControlledDynamics,
                          nonlinearSystem_.get(),
                          std::placeholders::_1,
                          std::placeholders::_2,
                          std::placeholders::_3,
                          std::placeholders::_4),
              std::bind(&AD_System_t::lift, nonlinearSystem_.get(), std::placeholders::_1),
              std::bind(&AD_System_t::retract, nonlinearSystem_.get(), std::placeholders::_1))
    {
    }

    //! copy constructor
    AutoDiffLinearizer(const AutoDiffLinearizer& arg)
        : Base(arg.nonlinearSystem_->getType()),
          dFdx_(arg.dFdx_),
          dFdu_(arg.dFdu_),
          nonlinearSystem_(arg.nonlinearSystem_->clone()),
          linearizer_(std::bind(&AD_System_t::computeControlledDynamics,
                          nonlinearSystem_.get(),
                          std::placeholders::_1,
                          std::placeholders::_2,
                          std::placeholders::_3,
                          std::placeholders::_4),
              std::bind(&AD_System_t::lift, nonlinearSystem_.get(), std::placeholders::_1),
              std::bind(&AD_System_t::retract, nonlinearSystem_.get(), std::placeholders::_1))
    {
    }

    //! destructor
    virtual ~AutoDiffLinearizer() {}
    //! deep cloning
    AutoDiffLinearizer<MANIFOLD, MANIFOLD_AD, CONTROL_DIM, CONT_T>* clone() const override
    {
        return new AutoDiffLinearizer<MANIFOLD, MANIFOLD_AD, CONTROL_DIM, CONT_T>(*this);
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
    virtual const state_matrix_t& getDerivativeState(const MANIFOLD& x,
        const control_vector_t& u,
        const Time_t tn = Time_t(0)) override
    {
        dFdx_ = linearizer_.getDerivativeState(x, u, tn);
        if (CONT_T == DISCRETE_TIME)
            dFdx_ += state_matrix_t::Identity();
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
    virtual const state_control_matrix_t& getDerivativeControl(const MANIFOLD& x,
        const control_vector_t& u,
        const Time_t tn = Time_t(0)) override
    {
        dFdu_ = linearizer_.getDerivativeControl(x, u, tn);
        return dFdu_;
    }

protected:
    state_matrix_t dFdx_;
    state_control_matrix_t dFdu_;

    std::shared_ptr<AD_System_t> nonlinearSystem_;  //!< instance of non-linear system

    linearizer_t linearizer_;  //!< instance of ad-linearizer
};

}  // namespace core
}  // namespace ct

#endif