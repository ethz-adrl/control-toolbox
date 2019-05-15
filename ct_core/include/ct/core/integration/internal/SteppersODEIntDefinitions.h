/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <boost/numeric/odeint.hpp>

namespace ct {
namespace core {
namespace internal {
/*****************************************************************************
 * Defining the (explicit) steppers
 *****************************************************************************/
//! Simple Euler stepper
template <size_t STATE_DIM, typename SCALAR = double>
using euler_t = boost::numeric::odeint::euler<Eigen::Matrix<SCALAR, STATE_DIM, 1>,
    SCALAR,
    Eigen::Matrix<SCALAR, STATE_DIM, 1>,
    SCALAR,
    boost::numeric::odeint::vector_space_algebra>;

//! Modified Midpoint stepper
template <size_t STATE_DIM, typename SCALAR = double>
using modified_midpoint_t = boost::numeric::odeint::modified_midpoint<Eigen::Matrix<SCALAR, STATE_DIM, 1>,
    SCALAR,
    Eigen::Matrix<SCALAR, STATE_DIM, 1>,
    SCALAR,
    boost::numeric::odeint::vector_space_algebra>;

//! Runge-Kutta4 stepper
template <size_t STATE_DIM, typename SCALAR = double>
using runge_kutta_4_t = boost::numeric::odeint::runge_kutta4<Eigen::Matrix<SCALAR, STATE_DIM, 1>,
    SCALAR,
    Eigen::Matrix<SCALAR, STATE_DIM, 1>,
    SCALAR,
    boost::numeric::odeint::vector_space_algebra>;

//! Runge-Kutta Dormand Price 5 stepper
template <size_t STATE_DIM, typename SCALAR = double>
using runge_kutta_dopri5_t = boost::numeric::odeint::runge_kutta_dopri5<Eigen::Matrix<SCALAR, STATE_DIM, 1>,
    SCALAR,
    Eigen::Matrix<SCALAR, STATE_DIM, 1>,
    SCALAR,
    boost::numeric::odeint::vector_space_algebra>;

//! Runge Kutta Fehlberg 78 stepper
template <size_t STATE_DIM, typename SCALAR = double>
using runge_kutta_fehlberg78_t = boost::numeric::odeint::runge_kutta_fehlberg78<Eigen::Matrix<SCALAR, STATE_DIM, 1>,
    SCALAR,
    Eigen::Matrix<SCALAR, STATE_DIM, 1>,
    Time,
    boost::numeric::odeint::vector_space_algebra>;

//! Bulirsch Stoer stepper
template <size_t STATE_DIM, typename SCALAR = double>
using bulirsch_stoer_t = boost::numeric::odeint::bulirsch_stoer<Eigen::Matrix<SCALAR, STATE_DIM, 1>,
    SCALAR,
    Eigen::Matrix<SCALAR, STATE_DIM, 1>,
    SCALAR,
    boost::numeric::odeint::vector_space_algebra>;

//! Adams Bashforth stepper
template <size_t STATE_DIM, size_t STEPS, typename SCALAR = double>
using adams_bashforth_uncontrolled_t = boost::numeric::odeint::adams_bashforth<STEPS,
    Eigen::Matrix<SCALAR, STATE_DIM, 1>,  // state
    SCALAR,                               // typename value
    Eigen::Matrix<SCALAR, STATE_DIM, 1>,  // derivative
    SCALAR,                               // typename time
    boost::numeric::odeint::vector_space_algebra>;

/*****************************************************************************
 * Defining the symplectic steppers
 *****************************************************************************/

//! A symplictic rk type stepper
template <size_t POS_DIM, size_t VEL_DIM, typename SCALAR = double>
using symplectic_rk_t = boost::numeric::odeint::symplectic_rkn_sb3a_mclachlan<Eigen::Matrix<SCALAR, POS_DIM, 1>,
    Eigen::Matrix<SCALAR, POS_DIM, 1>,
    SCALAR,
    Eigen::Matrix<SCALAR, POS_DIM, 1>,
    Eigen::Matrix<SCALAR, POS_DIM, 1>,
    SCALAR,
    boost::numeric::odeint::vector_space_algebra>;

// Symplectic euler stepper
template <size_t POS_DIM, size_t VEL_DIM, typename SCALAR = double>
using symplectic_euler_t = boost::numeric::odeint::symplectic_euler<Eigen::Matrix<SCALAR, POS_DIM, 1>,
    Eigen::Matrix<SCALAR, POS_DIM, 1>,
    SCALAR,
    Eigen::Matrix<SCALAR, POS_DIM, 1>,
    Eigen::Matrix<SCALAR, POS_DIM, 1>,
    SCALAR,
    boost::numeric::odeint::vector_space_algebra>;

/*****************************************************************************
 * Defining the (implicit) steppers
 *****************************************************************************/
// // works only for boost 1.56 or higher
//template <size_t STATE_DIM, size_t STEPS>
//using adams_bashforth_moulton_uncontrolled_t =
//		boost::numeric::odeint::adams_bashforth_moulton<
//		STEPS,
//		Eigen::Matrix<double, STATE_DIM, 1>,	// state
//		double,									// typename value
//		Eigen::Matrix<double, STATE_DIM, 1>,	// derivative
//		double, 								// typename time
//		boost::numeric::odeint::vector_space_algebra> ;

}  // namespace internal
}  // namespace core
}  // namespace ct
