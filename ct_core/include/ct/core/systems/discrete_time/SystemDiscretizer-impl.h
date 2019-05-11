/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#define SYMPLECTIC_ENABLED                   \
    template <size_t V, size_t P, size_t ST> \
    typename std::enable_if<(V > 0 && P > 0 && (V + P == ST)), void>::type
#define SYMPLECTIC_DISABLED                  \
    template <size_t V, size_t P, size_t ST> \
    typename std::enable_if<(V <= 0 || P <= 0 || (V + P != ST)), void>::type

namespace ct {
namespace core {

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
SystemDiscretizer<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::SystemDiscretizer()
    : cont_constant_controller_(new ConstantController<STATE_DIM, CONTROL_DIM, SCALAR>())
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
SystemDiscretizer<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::SystemDiscretizer(const SCALAR& dt,
    const ct::core::IntegrationType& integratorType,
    const int& K_sim)
    : dt_(dt),
      K_sim_(K_sim),
      integratorType_(integratorType),
      cont_constant_controller_(new ConstantController<STATE_DIM, CONTROL_DIM, SCALAR>())

{
    dt_sim_ = getSimulationTimestep();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
SystemDiscretizer<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::SystemDiscretizer(ContinuousSystemPtr system,
    const SCALAR& dt,
    const ct::core::IntegrationType& integratorType,
    const int& K_sim)
    : dt_(dt),
      K_sim_(K_sim),
      integratorType_(integratorType),
      cont_constant_controller_(new ConstantController<STATE_DIM, CONTROL_DIM, SCALAR>())

{
    dt_sim_ = getSimulationTimestep();
    changeContinuousTimeSystem(system);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
SystemDiscretizer<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::~SystemDiscretizer()
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
SystemDiscretizer<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::SystemDiscretizer(const SystemDiscretizer& arg)
    : dt_(arg.dt_), K_sim_(arg.K_sim_), dt_sim_(arg.dt_sim_), integratorType_(arg.integratorType_)
{
    changeContinuousTimeSystem(ContinuousSystemPtr(cont_time_system_->clone()));
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
SystemDiscretizer<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>*
SystemDiscretizer<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::clone() const
{
    return new SystemDiscretizer(*this);
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
SCALAR SystemDiscretizer<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getSimulationTimestep()
{
    return dt_ / (SCALAR)K_sim_;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void SystemDiscretizer<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::setIntegrationType(
    const ct::core::IntegrationType& integratorType)
{
    integratorType_ = integratorType;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void SystemDiscretizer<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::setParameters(const SCALAR& dt, const int& K_sim)
{
    dt_ = dt;
    K_sim_ = K_sim;
    dt_sim_ = getSimulationTimestep();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void SystemDiscretizer<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::initialize()
{
    substepRecorder_ =
        SubstepRecorderPtr(new ct::core::SubstepRecorder<STATE_DIM, CONTROL_DIM, SCALAR>(cont_time_system_));

    if (integratorType_ != ct::core::IntegrationType::EULER_SYM && integratorType_ != ct::core::IntegrationType::RK_SYM)
    {
        integrator_ = std::shared_ptr<ct::core::Integrator<STATE_DIM, SCALAR>>(
            new ct::core::Integrator<STATE_DIM, SCALAR>(cont_time_system_, integratorType_, substepRecorder_));
    }
    initializeSymplecticIntegrator<V_DIM, P_DIM, STATE_DIM>();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void SystemDiscretizer<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::changeContinuousTimeSystem(
    ContinuousSystemPtr dyn)
{
    cont_time_system_ = std::shared_ptr<ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>>(dyn->clone());
    cont_time_system_->setController(cont_constant_controller_);
    initialize();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void SystemDiscretizer<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::propagateControlledDynamics(
    const StateVector<STATE_DIM, SCALAR>& state,
    const time_t n,
    const ControlVector<CONTROL_DIM, SCALAR>& control,
    StateVector<STATE_DIM, SCALAR>& stateNext)
{
    assert(K_sim_ > 0);
    assert(dt_sim_ > 0.0);
    assert(dt_ > 0.0);
    assert(cont_time_system_ != nullptr);

    cont_constant_controller_->setControl(control);

    // reset substep recorder for every new control step
    substepRecorder_->reset();

    // initialize state to propagate
    stateNext = state;

    // perform integration
    if (integratorType_ == ct::core::IntegrationType::EULER_SYM || integratorType_ == ct::core::IntegrationType::RK_SYM)
    {
        integrateSymplectic<V_DIM, P_DIM, STATE_DIM>(stateNext, n * dt_, K_sim_, dt_sim_);
    }
    else
    {
        integrator_->integrate_n_steps(stateNext, n * dt_, K_sim_, dt_sim_);
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
                    substepRecorder_));

        //! initialize RK symplectic integrator
        integratorRkSymplectic_ = std::shared_ptr<ct::core::IntegratorSymplecticRk<P_DIM, V_DIM, CONTROL_DIM, SCALAR>>(
            new ct::core::IntegratorSymplecticRk<P_DIM, V_DIM, CONTROL_DIM, SCALAR>(
                std::static_pointer_cast<ct::core::SymplecticSystem<P_DIM, V_DIM, CONTROL_DIM, SCALAR>>(
                    cont_time_system_)));
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

    if (integratorType_ == ct::core::IntegrationType::EULER_SYM)
    {
        integratorEulerSymplectic_->integrate_n_steps(x0, t, steps, dt_sim);
    }
    else if (integratorType_ == ct::core::IntegrationType::RK_SYM)
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

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
const std::shared_ptr<ct::core::StateVectorArray<STATE_DIM, SCALAR>>&
SystemDiscretizer<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getSubstates() const
{
    return substepRecorder_->getSubstates();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
const std::shared_ptr<ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>>&
SystemDiscretizer<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getSubcontrols() const
{
    return substepRecorder_->getSubcontrols();
}


}  // namespace core
}  // namespace ct

#undef SYMPLECTIC_ENABLED
#undef SYMPLECTIC_DISABLED
