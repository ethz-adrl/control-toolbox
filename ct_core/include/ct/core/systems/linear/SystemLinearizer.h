/**********************************************************************************************************************
This file is part of the Control Toobox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once


#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <ct/core/types/StateVector.h>
#include <ct/core/types/ControlVector.h>

#include <ct/core/systems/ControlledSystem.h>

#include "LinearSystem.h"

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
 * @tparam dimension of state vector
 * @tparam dimension of control vector
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class SystemLinearizer : public LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef StateVector<STATE_DIM, SCALAR> state_vector_t;        //!< state vector type
    typedef ControlVector<CONTROL_DIM, SCALAR> control_vector_t;  //!< control vector type

    typedef Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM> state_matrix_t;            //!< state Jacobian type (A)
    typedef Eigen::Matrix<SCALAR, STATE_DIM, CONTROL_DIM> state_control_matrix_t;  //! control Jacobian type (B)

    //! default constructor
    /*!
	 * Initializes the linearizer with a non-linear system.
	 *
	 * @param nonlinearSystem non-linear system to linearize
	 * @param doubleSidedDerivative if true, double sided numerical differentiation is used
	 */
    SystemLinearizer(std::shared_ptr<ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>> nonlinearSystem,
        bool doubleSidedDerivative = true)
        : LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>(nonlinearSystem->getType()),
          nonlinearSystem_(nonlinearSystem),
          doubleSidedDerivative_(doubleSidedDerivative)
    {
        if (nonlinearSystem == nullptr)
            throw std::runtime_error("SystemLinearizer: Nonlinear system is nullptr!");

        if (nonlinearSystem_->getType() == SECOND_ORDER)
            isSecondOrderSystem_ = true;
        else
            isSecondOrderSystem_ = false;

        dFdx_.setZero();
        dFdu_.setZero();

        if (isSecondOrderSystem_)
        {
            // fill default
            dFdx_.template topLeftCorner<STATE_DIM / 2, STATE_DIM / 2>().setZero();
            dFdx_.template topRightCorner<STATE_DIM / 2, STATE_DIM / 2>().setIdentity();

            // fill default
            dFdu_.template topRows<STATE_DIM / 2>().setZero();
        }

        eps_ = sqrt(Eigen::NumTraits<SCALAR>::epsilon());
    }

    //! copy constructor
    SystemLinearizer(const SystemLinearizer& arg)
        : LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>(arg),
          nonlinearSystem_(arg.nonlinearSystem_->clone()),
          dFdx_(arg.dFdx_),
          dFdu_(arg.dFdu_),
          dxdt_ref_(arg.dxdt_ref_),
          eps_(arg.eps_),
          doubleSidedDerivative_(arg.doubleSidedDerivative_),
          isSecondOrderSystem_(arg.getType() == SECOND_ORDER)
    {
    }

    //! destructor
    virtual ~SystemLinearizer() {}
    //! deep cloning
    SystemLinearizer<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override
    {
        return new SystemLinearizer<STATE_DIM, CONTROL_DIM, SCALAR>(*this);
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
    virtual const state_matrix_t& getDerivativeState(const state_vector_t& x,
        const control_vector_t& u,
        const SCALAR t = SCALAR(0.0)) override
    {
        nonlinearSystem_->computeControlledDynamics(x, t, u, dxdt_ref_);

        for (size_t i = 0; i < STATE_DIM; ++i)
        {
            state_vector_t dxdt;

            // inspired from http://en.wikipedia.org/wiki/Numerical_differentiation#Practical_considerations_using_floating_point_arithmetic
            SCALAR h = eps_ * std::max(fabs(x(i)), SCALAR(1.0));
            SCALAR x_ph = x(i) + h;
            SCALAR dxp = x_ph - x(i);

            state_vector_t x_perturbed = x;
            x_perturbed(i) = x_ph;

            // get evaluation of f(x,u)
            nonlinearSystem_->computeControlledDynamics(x_perturbed, t, u, dxdt);

            if (doubleSidedDerivative_)
            {
                state_vector_t dxdt_low;

                SCALAR x_mh = x(i) - h;
                SCALAR dxm = x(i) - x_mh;

                x_perturbed = x;
                x_perturbed(i) = x_mh;
                nonlinearSystem_->computeControlledDynamics(x_perturbed, t, u, dxdt_low);

                if (isSecondOrderSystem_)
                {
                    dFdx_.template block<STATE_DIM / 2, 1>(STATE_DIM / 2, i) =
                        (dxdt.template segment<STATE_DIM / 2>(STATE_DIM / 2) -
                            dxdt_low.template segment<STATE_DIM / 2>(STATE_DIM / 2)) /
                        (dxp + dxm);
                }
                else
                {
                    dFdx_.template col(i) = (dxdt - dxdt_low) / (dxp + dxm);
                }
            }
            else
            {
                if (isSecondOrderSystem_)
                {
                    dFdx_.template block<STATE_DIM / 2, 1>(STATE_DIM / 2, i) =
                        (dxdt.template segment<STATE_DIM / 2>(STATE_DIM / 2) -
                            dxdt_ref_.template segment<STATE_DIM / 2>(STATE_DIM / 2)) /
                        dxp;
                }
                else
                {
                    dFdx_.template col(i) = (dxdt - dxdt_ref_) / dxp;
                }
            }
        }

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
    virtual const state_control_matrix_t& getDerivativeControl(const state_vector_t& x,
        const control_vector_t& u,
        const SCALAR t = SCALAR(0.0)) override
    {
        nonlinearSystem_->computeControlledDynamics(x, t, u, dxdt_ref_);

        for (size_t i = 0; i < CONTROL_DIM; ++i)
        {
            state_vector_t dxdt;

            // inspired from http://en.wikipedia.org/wiki/Numerical_differentiation#Practical_considerations_using_floating_point_arithmetic
            SCALAR h = eps_ * std::max(fabs(u(i)), SCALAR(1.0));
            SCALAR u_ph = u(i) + h;
            SCALAR dup = u_ph - u(i);

            control_vector_t u_perturbed = u;
            u_perturbed(i) = u_ph;

            // get evaluation of f(x,u)
            nonlinearSystem_->computeControlledDynamics(x, t, u_perturbed, dxdt);

            if (doubleSidedDerivative_)
            {
                state_vector_t dxdt_low;

                SCALAR u_mh = u(i) - h;
                SCALAR dum = u_ph - u(i);

                u_perturbed = u;
                u_perturbed(i) = u_mh;

                nonlinearSystem_->computeControlledDynamics(x, t, u_perturbed, dxdt_low);

                if (isSecondOrderSystem_)
                {
                    dFdu_.template block<STATE_DIM / 2, 1>(STATE_DIM / 2, i) =
                        (dxdt.template segment<STATE_DIM / 2>(STATE_DIM / 2) -
                            dxdt_low.template segment<STATE_DIM / 2>(STATE_DIM / 2)) /
                        (dup + dum);
                }
                else
                {
                    dFdu_.template col(i) = (dxdt - dxdt_low) / (dup + dum);
                }
            }
            else
            {
                if (isSecondOrderSystem_)
                {
                    dFdu_.template block<STATE_DIM / 2, 1>(STATE_DIM / 2, i) =
                        (dxdt.template segment<STATE_DIM / 2>(STATE_DIM / 2) -
                            dxdt_ref_.template segment<STATE_DIM / 2>(STATE_DIM / 2)) /
                        dup;
                }
                else
                {
                    dFdu_.template col(i) = (dxdt - dxdt_ref_) / dup;
                }
            }
        }

        return dFdu_;
    }


protected:
    std::shared_ptr<ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>>
        nonlinearSystem_;  //!< instance of non-linear system

    state_matrix_t dFdx_;          //!< Jacobian wrt state
    state_control_matrix_t dFdu_;  //!< Jacobian wrt input

    StateVector<STATE_DIM, SCALAR> dxdt_ref_;  //!< reference state for numerical differentiation


    SCALAR eps_;  //!< perturbation for numerical differentiation

    bool doubleSidedDerivative_;  //!< flag if double sided numerical differentiation should be used

    bool isSecondOrderSystem_;  //!< flag if system is a second order system
};


}  // core
}  // ct
