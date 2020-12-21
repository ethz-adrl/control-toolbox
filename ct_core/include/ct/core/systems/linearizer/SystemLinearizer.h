/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/systems/LinearSystem.h>
#include <ct/core/systems/linearizer/DynamicsLinearizerNumDiff.h>

namespace ct {
namespace core {

//! Computes the linearization of a general non-linear ControlledSystem using numerical differentiation
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
 * In case the ControlledSystem is a pure second-order system, the upper half of A is not explicitely computed
 * but A is assumed to be of the following form
 *
 * \f[
 *  A =
 *  \begin{bmatrix}
 *  	0 & I \\
 *  	... & ...
 *  \end{bmatrix}
 * \f]
 *
 * Examples for using the SystemLinearizer (and the Auto-diff Linearizer) can be found in \ref AutoDiffLinearizerTest.cpp
 *
 * \note In case your ControlledSystem is templated on scalar type, we suggest using the ADCodegenLinearizer
 * for highest efficiency and accuracy. If this is not the case but your system is a RigidBodySystem you can fall back
 * to the ct::rbd::RBDLinearizer for good accuracy and speed.
 *
 * @tparam STATE_DIM   dimension of state vector
 * @tparam CONTROL_DIM dimension of control vector
 * @tparam SCALAR underlying scalar type of the system
 */
template <typename MANIFOLD, size_t CONTROL_DIM, bool CONT_T>
class SystemLinearizer : public LinearSystem<MANIFOLD, CONTROL_DIM, CONT_T>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const size_t STATE_DIM = MANIFOLD::TangentDim;
    using SCALAR = typename MANIFOLD::Scalar;
    using Tangent = typename MANIFOLD::Tangent;
    using NonlinearSystem_t = ControlledSystem<MANIFOLD, CONTROL_DIM, CONT_T>;  //!< type of system to be linearized
    using Base = LinearSystem<MANIFOLD, CONTROL_DIM, CONT_T>;                   //!< Base class type
    using Time_t = typename Base::Time_t;                                       //!< Time type as defined in System
    using control_vector_t = typename Base::control_vector_t;                   //!< input vector type
    using state_matrix_t = typename Base::state_matrix_t;                       //!< state Jacobian type
    using state_control_matrix_t = typename Base::state_control_matrix_t;       //!< input Jacobian type

    //! default constructor
    /*!
	 * Initializes the linearizer with a non-linear system.
	 *
	 * @param nonlinearSystem non-linear system to linearize
	 * @param doubleSidedDerivative if true, double sided numerical differentiation is used
	 */
    SystemLinearizer(std::shared_ptr<NonlinearSystem_t> nonlinearSystem, bool doubleSidedDerivative = true)
        : Base(nonlinearSystem->getType()),
          nonlinearSystem_(nonlinearSystem),
          linearizer_(std::bind(&NonlinearSystem_t::computeControlledDynamics,
                          nonlinearSystem_.get(),
                          std::placeholders::_1,
                          std::placeholders::_2,
                          std::placeholders::_3,
                          std::placeholders::_4),
              std::bind(&NonlinearSystem_t::lift, nonlinearSystem_.get(), std::placeholders::_1),
              std::bind(&NonlinearSystem_t::retract, nonlinearSystem_.get(), std::placeholders::_1),
              doubleSidedDerivative)
    {
        if (nonlinearSystem == nullptr)
            throw std::runtime_error("SystemLinearizer: Nonlinear system is nullptr!");

        if (nonlinearSystem->getType() == SECOND_ORDER)
        {
            if (CONT_T == DISCRETE_TIME)
            {
                throw std::runtime_error(
                    "The System Linearizer does currently not support second-order discrete-time systems.");
                // TODO: implement this properly
            }
            isSecondOrderSystem_ = true;
        }
        else
            isSecondOrderSystem_ = false;

        dFdx_.setZero();
        dFdu_.setZero();
    }

    //! copy constructor
    SystemLinearizer(const SystemLinearizer& arg)
        : Base(arg),
          nonlinearSystem_(arg.nonlinearSystem_->clone()),
          linearizer_(std::bind(&NonlinearSystem_t::computeControlledDynamics,
                          nonlinearSystem_.get(),
                          std::placeholders::_1,
                          std::placeholders::_2,
                          std::placeholders::_3,
                          std::placeholders::_4),
              std::bind(&NonlinearSystem_t::lift, nonlinearSystem_.get(), std::placeholders::_1),
              std::bind(&NonlinearSystem_t::retract, nonlinearSystem_.get(), std::placeholders::_1),
              arg.linearizer_.getDoubleSidedDerivativeFlag()),
          dFdx_(arg.dFdx_),
          dFdu_(arg.dFdu_),
          isSecondOrderSystem_(arg.getType() == SECOND_ORDER)
    {
    }

    //! deep cloning
    SystemLinearizer<MANIFOLD, CONTROL_DIM, CONT_T>* clone() const override
    {
        return new SystemLinearizer<MANIFOLD, CONTROL_DIM, CONT_T>(*this);
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
    virtual const state_matrix_t& getDerivativeState(const MANIFOLD& m,
        const control_vector_t& u,
        const Time_t t = 0.0) override
    {
        dFdx_ = linearizer_.getDerivativeState(m, u, t);

        if (isSecondOrderSystem_)
        {
            dFdx_.template topLeftCorner<STATE_DIM / 2, STATE_DIM / 2>().setZero();
            dFdx_.template topRightCorner<STATE_DIM / 2, STATE_DIM / 2>().setIdentity();
        }

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
    virtual const state_control_matrix_t& getDerivativeControl(const MANIFOLD& m,
        const control_vector_t& u,
        const Time_t t = 0.0) override
    {
        dFdu_ = linearizer_.getDerivativeControl(m, u, t);

        if (isSecondOrderSystem_)
        {
            dFdu_.template topRows<STATE_DIM / 2>().setZero();
        }

        return dFdu_;
    }

protected:
    //!< instance of non-linear system, living on MANIFOLD
    std::shared_ptr<NonlinearSystem_t> nonlinearSystem_;

    //!< instance of numerical-linearizer
    DynamicsLinearizerNumDiff<MANIFOLD, CONTROL_DIM, typename Base::Time_t> linearizer_;

    //!< Jacobian wrt state
    state_matrix_t dFdx_;

    //!< Jacobian wrt input
    state_control_matrix_t dFdu_;

    //!< flag if system is a second order system
    bool isSecondOrderSystem_;
};

}  // namespace core
}  // namespace ct
