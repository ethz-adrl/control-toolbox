/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#define SYMPLECTIC_ENABLED        \
    template <size_t V, size_t P> \
    typename std::enable_if<(V > 0 && P > 0), void>::type
#define SYMPLECTIC_DISABLED       \
    template <size_t V, size_t P> \
    typename std::enable_if<(V <= 0 || P <= 0), void>::type

namespace ct {
namespace core {


todo changeNonlinearSystem(dyn)
{
	cont_time_system_ = std::shared_ptr<ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>>(dyn->clone());
	cont_time_system_->setController()

    substepRecorder_ = SubstepRecorderPtr(
        new ct::core::SubstepRecorder<STATE_DIM, CONTROL_DIM, SCALAR>(cont_time_system_));

    // if symplectic integrator then don't create normal ones
    if (settings_.integrator != ct::core::IntegrationType::EULER_SYM &&
        settings_.integrator != ct::core::IntegrationType::RK_SYM)
    {
        integrator_ = std::shared_ptr<ct::core::Integrator<STATE_DIM, SCALAR>>(
            new ct::core::Integrator<STATE_DIM, SCALAR>(cont_time_system_, settings_.integrator, substepRecorder_));
    }
    initializeSymplecticIntegrator<V_DIM, P_DIM>();
}


todo propagate()
{
	// for each control step
	for ...
	{
	// reset substep recorder for every new control step
	// compare NLOCBackend
    substepRecorder_->reset();

    // do some integration

    // get substeps for sensitivities
    substepsX = substepRecorder_->getSubstates();
    substepsU = substepRecorder_->getSubcontrols();

	}
}



template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
SYMPLECTIC_ENABLED SystemDiscretizer<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::initializeSymplecticIntegrator()
{
    if (cont_time_system_->isSymplectic())
    {
        //! it only makes sense to compile the following code, if V_DIM > 0 and P_DIM > 0
        //! initialize symplectic Euler integrator
        integratorEulerSymplectic_ =
            std::shared_ptr<ct::core::IntegratorSymplecticEuler<P_DIM, V_DIM, CONTROL_DIM, SCALAR>>(
                new ct::core::IntegratorSymplecticEuler<P_DIM, V_DIM, CONTROL_DIM, SCALAR>(
                    std::static_pointer_cast<ct::core::SymplecticSystem<P_DIM, V_DIM, CONTROL_DIM, SCALAR>>(
                        cont_time_system_),
                    substepRecorders_));

        //! initialize RK symplectic integrator
        integratorRkSymplectic_[i] =
            std::shared_ptr<ct::core::IntegratorSymplecticRk<P_DIM, V_DIM, CONTROL_DIM, SCALAR>>(
                new ct::core::IntegratorSymplecticRk<P_DIM, V_DIM, CONTROL_DIM, SCALAR>(
                    std::static_pointer_cast<ct::core::SymplecticSystem<P_DIM, V_DIM, CONTROL_DIM, SCALAR>>(
                        cont_time_system_)));
        //! todo: does not yet support substep-recorder?
    }
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
SYMPLECTIC_DISABLED SystemDiscretizer<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::initializeSymplecticIntegrator()
{
    // leave empty
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
SYMPLECTIC_ENABLED SystemDiscretizer<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::integrateSymplectic(
    ct::core::StateVector<STATE_DIM, SCALAR>& x0,
    const double& t,
    const size_t& steps,
    const double& dt_sim) const
{
    if (!cont_time_system_->isSymplectic())
        throw std::runtime_error("Trying to integrate using symplectic integrator, but system is not symplectic.");

    if (settings_.integrator == ct::core::IntegrationType::EULER_SYM)
    {
        integratorEulerSymplectic_->integrate_n_steps(x0, t, steps, dt_sim);
    }
    else if (settings_.integrator == ct::core::IntegrationType::RK_SYM)
    {
        integratorRkSymplectic_->integrate_n_steps(x0, t, steps, dt_sim);
    }
    else
    {
        throw std::runtime_error("invalid symplectic integrator specified");
    }
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
SYMPLECTIC_DISABLED SystemDiscretizer<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::integrateSymplectic(
    ct::core::StateVector<STATE_DIM, SCALAR>& x0,
    const double& t,
    const size_t& steps,
    const double& dt_sim) const
{
    throw std::runtime_error("Symplectic integrator selected but invalid dimensions for it. Check V_DIM>1, P_DIM>1");
}

}  // namespace core
}  // namespace ct

#undef SYMPLECTIC_ENABLED
#undef SYMPLECTIC_DISABLED
