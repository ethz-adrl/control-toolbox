/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "DiscreteControlledSystem.h"
#include "../continuous_time/ControlledSystem.h"

#include <ct/core/integration/Integrator.h>
#include <ct/core/integration/IntegratorSymplectic.h>

#define SYMPLECTIC_ENABLED        \
    template <size_t V, size_t P> \
    typename std::enable_if<(V > 0 && P > 0), void>::type
#define SYMPLECTIC_DISABLED       \
    template <size_t V, size_t P> \
    typename std::enable_if<(V <= 0 || P <= 0), void>::type


namespace ct {
namespace core {

//! Discretize a general, continuous-time non-linear dynamic system using forward integration
/*!
 * The SystemDiscretizer transforms a continuous-time system into a discrete-time system by forward integration.
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR = double>
class SystemDiscretizer : public DiscreteControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    // convenience typedefs for the different integrator classes CT currently has to offer
    using IntegratorPtr = std::shared_ptr<Integrator<STATE_DIM, SCALAR>>;
    using IntegratorSymplecticEulerPtr =
        std::shared_ptr<ct::core::IntegratorSymplecticEuler<P_DIM, V_DIM, CONTROL_DIM, SCALAR>>;
    using IntegratorSymplecticRkPtr =
        std::shared_ptr<ct::core::IntegratorSymplecticRk<P_DIM, V_DIM, CONTROL_DIM, SCALAR>>;

    typedef std::shared_ptr<ct::core::SubstepRecorder<STATE_DIM, CONTROL_DIM, SCALAR>> SubstepRecorderPtr;

    todo changeNonlinearSystem();

protected:
    SYMPLECTIC_ENABLED initializeSymplecticIntegrator();

    SYMPLECTIC_DISABLED initializeSymplecticIntegrator();

    SYMPLECTIC_ENABLED integrateSymplectic(ct::core::StateVector<STATE_DIM, SCALAR>& x0,
        const double& t,
        const size_t& steps,
        const double& dt_sim) const;

    SYMPLECTIC_DISABLED integrateSymplectic(ct::core::StateVector<STATE_DIM, SCALAR>& x0,
        const double& t,
        const size_t& steps,
        const double& dt_sim) const;

    //! the time discretization interval
    SCALAR dt_;

    //! the forward simulation can be discretized finer than dt_. K_sim_ is an integer number representing the number of simulation sub-steps.
    int K_sim_;

    //! the continuous-time system to be discretized
    std::shared_ptr<ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>> cont_time_system_;

    //! an integrator for forward integrating a general continuous-time system with standard RK methods
    IntegratorPtr integrator_;

    //! an integrator for forward integrating a symplectic continuous-time system with symplectic Euler
    IntegratorSymplecticEulerPtr integratorEulerSymplectic_;

    //! an integrator for forward integrating a symplectic continuous-time system with symplectic RK methods
    IntegratorSymplecticRkPtr integratorsRkSymplectic_;

    //! substep recorder
    SubstepRecorderPtr substepRecorder_;

};


}  // namespace core
}  // namespace ct

#undef SYMPLECTIC_ENABLED
#undef SYMPLECTIC_DISABLED
