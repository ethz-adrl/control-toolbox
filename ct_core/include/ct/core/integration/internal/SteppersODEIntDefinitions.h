/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/types/TypeTraits.h>
#include <boost/numeric/odeint.hpp>
#include "manif_operations.h"

namespace ct {
namespace core {
namespace internal {
/*****************************************************************************
 * Defining the (explicit) steppers
 *****************************************************************************/

/**
 * @brief this macro selects an "operations-class" for the boost odeint steppers
 * Based on the MANIFOLD in use, it either selects the default_operations by boost for the 
 * Euclidean case, or the custom CT implementation for manif operations.
 */
#define SELECT_OPERATIONS(MANIFOLD)                                                                      \
    typename std::conditional<is_euclidean<MANIFOLD>::value, boost::numeric::odeint::default_operations, \
        boost::numeric::odeint::manif_operations>::type

//! Simple Euler stepper, uses specialized operations set based on the manifold in use.
template <typename MANIFOLD>
using euler_t = boost::numeric::odeint::euler<MANIFOLD,
    typename MANIFOLD::Scalar,
    typename MANIFOLD::Tangent,
    typename MANIFOLD::Scalar,
    boost::numeric::odeint::vector_space_algebra,
    SELECT_OPERATIONS(MANIFOLD)>;

//! Modified Midpoint stepper
template <typename MANIFOLD>
using modified_midpoint_t = boost::numeric::odeint::modified_midpoint<MANIFOLD,
    typename MANIFOLD::Scalar,
    typename MANIFOLD::Tangent,
    typename MANIFOLD::Scalar,
    boost::numeric::odeint::vector_space_algebra,
    SELECT_OPERATIONS(MANIFOLD)>;

//! Runge-Kutta4 stepper
template <typename MANIFOLD>
using runge_kutta_4_t = boost::numeric::odeint::runge_kutta4<MANIFOLD,
    typename MANIFOLD::Scalar,
    typename MANIFOLD::Tangent,
    typename MANIFOLD::Scalar,
    boost::numeric::odeint::vector_space_algebra,
    SELECT_OPERATIONS(MANIFOLD)>;

//! Runge-Kutta Dormand Price 5 stepper
template <typename MANIFOLD>
using runge_kutta_dopri5_t = boost::numeric::odeint::runge_kutta_dopri5<MANIFOLD,
    typename MANIFOLD::Scalar,
    typename MANIFOLD::Tangent,
    typename MANIFOLD::Scalar,
    boost::numeric::odeint::vector_space_algebra,
    SELECT_OPERATIONS(MANIFOLD)>;

//! Runge Kutta Fehlberg 78 stepper
template <typename MANIFOLD>
using runge_kutta_fehlberg78_t = boost::numeric::odeint::runge_kutta_fehlberg78<MANIFOLD,
    typename MANIFOLD::Scalar,
    typename MANIFOLD::Tangent,
    Time,
    boost::numeric::odeint::vector_space_algebra,
    SELECT_OPERATIONS(MANIFOLD)>;

//! Bulirsch Stoer stepper
template <typename MANIFOLD>
using bulirsch_stoer_t = boost::numeric::odeint::bulirsch_stoer<MANIFOLD,
    typename MANIFOLD::Scalar,
    typename MANIFOLD::Tangent,
    typename MANIFOLD::Scalar,
    boost::numeric::odeint::vector_space_algebra,
    SELECT_OPERATIONS(MANIFOLD)>;

//! Adams Bashforth stepper
template <typename MANIFOLD, size_t STEPS>
using adams_bashforth_uncontrolled_t = boost::numeric::odeint::adams_bashforth<STEPS,
    MANIFOLD,
    typename MANIFOLD::Scalar,
    typename MANIFOLD::Tangent,
    typename MANIFOLD::Scalar,
    boost::numeric::odeint::vector_space_algebra,
    SELECT_OPERATIONS(MANIFOLD)>;

/*****************************************************************************
 * Defining the symplectic steppers
 *****************************************************************************/

//! A symplectic rk type stepper
template <typename SYM_MFD>
using symplectic_rk_t = boost::numeric::odeint::symplectic_rkn_sb3a_mclachlan<typename SYM_MFD::PosTangent,
    typename SYM_MFD::PosTangent,
    typename SYM_MFD::Scalar,
    typename SYM_MFD::PosTangent,
    typename SYM_MFD::PosTangent,
    typename SYM_MFD::Scalar,
    boost::numeric::odeint::vector_space_algebra,
    SELECT_OPERATIONS(SYM_MFD)>;

// Symplectic euler stepper
template <typename SYM_MFD>
using symplectic_euler_t = boost::numeric::odeint::symplectic_euler<typename SYM_MFD::PosTangent,
    typename SYM_MFD::PosTangent,
    typename SYM_MFD::Scalar,
    typename SYM_MFD::PosTangent,
    typename SYM_MFD::PosTangent,
    typename SYM_MFD::Scalar,
    boost::numeric::odeint::vector_space_algebra,
    SELECT_OPERATIONS(SYM_MFD)>;

/****************************************************************************
 Defining the (implicit) steppers
****************************************************************************/
// works only for boost 1.56 or higher
template <typename MANIFOLD, size_t STEPS>
using adams_bashforth_moulton_uncontrolled_t = boost::numeric::odeint::adams_bashforth_moulton<STEPS,
    MANIFOLD,                    // state
    double,                      // typename value
    typename MANIFOLD::Tangent,  // derivative
    double,                      // typename time
    boost::numeric::odeint::vector_space_algebra,
    SELECT_OPERATIONS(MANIFOLD)>;

}  // namespace internal
}  // namespace core
}  // namespace ct
