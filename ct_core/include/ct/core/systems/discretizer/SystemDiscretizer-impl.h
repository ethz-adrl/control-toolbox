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

template <typename MNF, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM>
SystemDiscretizer<MNF, CONTROL_DIM, P_DIM, V_DIM>::SystemDiscretizer()
    : cont_constant_controller_(new ConstantController<MNF, CONTROL_DIM, CONTINUOUS_TIME>())
{
}

template <typename MNF, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM>
SystemDiscretizer<MNF, CONTROL_DIM, P_DIM, V_DIM>::SystemDiscretizer(const SCALAR& dt,
    const IntegrationType& integratorType,
    const int& K_sim)
    : dt_(dt),
      K_sim_(K_sim),
      integratorType_(integratorType),
      cont_constant_controller_(new ConstantController<MNF, CONTROL_DIM, CONTINUOUS_TIME>())

{
    dt_sim_ = getSimulationTimestep();
}

template <typename MNF, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM>
SystemDiscretizer<MNF, CONTROL_DIM, P_DIM, V_DIM>::SystemDiscretizer(ContinuousSystemPtr system,
    const SCALAR& dt,
    const IntegrationType& integratorType,
    const int& K_sim)
    : dt_(dt),
      K_sim_(K_sim),
      integratorType_(integratorType),
      cont_constant_controller_(new ConstantController<MNF, CONTROL_DIM, CONTINUOUS_TIME>())

{
    dt_sim_ = getSimulationTimestep();
    changeContinuousTimeSystem(system);
}

template <typename MNF, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM>
SystemDiscretizer<MNF, CONTROL_DIM, P_DIM, V_DIM>::~SystemDiscretizer()
{
}

template <typename MNF, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM>
SystemDiscretizer<MNF, CONTROL_DIM, P_DIM, V_DIM>::SystemDiscretizer(const SystemDiscretizer& arg)
    : dt_(arg.dt_), K_sim_(arg.K_sim_), dt_sim_(arg.dt_sim_), integratorType_(arg.integratorType_)
{
    changeContinuousTimeSystem(ContinuousSystemPtr(cont_time_system_->clone()));
}

template <typename MNF, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM>
SystemDiscretizer<MNF, CONTROL_DIM, P_DIM, V_DIM>* SystemDiscretizer<MNF, CONTROL_DIM, P_DIM, V_DIM>::clone() const
{
    return new SystemDiscretizer(*this);
}

template <typename MNF, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM>
auto SystemDiscretizer<MNF, CONTROL_DIM, P_DIM, V_DIM>::getSimulationTimestep() -> SCALAR
{
    return dt_ / (SCALAR)K_sim_;
}

template <typename MNF, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM>
void SystemDiscretizer<MNF, CONTROL_DIM, P_DIM, V_DIM>::setIntegrationType(const IntegrationType& integratorType)
{
    integratorType_ = integratorType;
}

template <typename MNF, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM>
void SystemDiscretizer<MNF, CONTROL_DIM, P_DIM, V_DIM>::setParameters(const SCALAR& dt, const int& K_sim)
{
    dt_ = dt;
    K_sim_ = K_sim;
    dt_sim_ = getSimulationTimestep();
}

template <typename MNF, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM>
void SystemDiscretizer<MNF, CONTROL_DIM, P_DIM, V_DIM>::initialize()
{
    substepRecorder_ = SubstepRecorderPtr(new SubstepRecorder<MNF, CONTROL_DIM>(cont_time_system_));

    if (integratorType_ != IntegrationType::EULER_SYM && integratorType_ != IntegrationType::RK_SYM)
    {
        integrator_ =
            std::shared_ptr<Integrator<MNF>>(new Integrator<MNF>(cont_time_system_, integratorType_, substepRecorder_));
    }
    //initializeSymplecticIntegrator<V_DIM, P_DIM, STATE_DIM>(); // TODO: bring back
}

template <typename MNF, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM>
void SystemDiscretizer<MNF, CONTROL_DIM, P_DIM, V_DIM>::changeContinuousTimeSystem(ContinuousSystemPtr dyn)
{
    cont_time_system_ = ContinuousSystemPtr(dyn->clone());
    cont_time_system_->setController(cont_constant_controller_);
    initialize();
}

template <typename MNF, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM>
void SystemDiscretizer<MNF, CONTROL_DIM, P_DIM, V_DIM>::computeControlledDynamics(const MNF& state,
    const Time_t& n,
    const ControlVector<CONTROL_DIM, SCALAR>& control,
    typename MNF::Tangent& m_incr)
{
    assert(K_sim_ > 0);
    assert(dt_sim_ > 0.0);
    assert(dt_ > 0.0);
    assert(cont_time_system_ != nullptr);

    cont_constant_controller_->setControl(control);

    // reset substep recorder for every new control step
    substepRecorder_->reset();

    // initialize state to propagate
    MNF stateNext = state;

    // perform integration // TODO: bring back symplectic stuff
    // if (integratorType_ ==  IntegrationType::EULER_SYM || integratorType_ ==  IntegrationType::RK_SYM)
    // {
    //     integrateSymplectic<V_DIM, P_DIM, STATE_DIM>(stateNext, n * dt_, K_sim_, dt_sim_);
    // }
    // else
    // {
    integrator_->integrate_n_steps(stateNext, n * dt_, K_sim_, dt_sim_);
    //}

    m_incr = stateNext - state;  // TODO: verify this for manifold case
}

/*
template <typename MNF, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM>
SYMPLECTIC_ENABLED SystemDiscretizer<MNF, CONTROL_DIM, P_DIM, V_DIM>::initializeSymplecticIntegrator()
{
    if (cont_time_system_->isSymplectic())
    {
        //! it only makes sense to compile the following code, if V_DIM > 0 and P_DIM > 0
        //! initialize symplectic Euler integrator
        integratorEulerSymplectic_ =
            std::shared_ptr< IntegratorSymplecticEuler<P_DIM, V_DIM, CONTROL_DIM, SCALAR>>(
                new  IntegratorSymplecticEuler<P_DIM, V_DIM, CONTROL_DIM, SCALAR>(
                    std::static_pointer_cast< SymplecticSystem<P_DIM, V_DIM, CONTROL_DIM, SCALAR>>(
                        cont_time_system_),
                    substepRecorder_));

        //! initialize RK symplectic integrator
        integratorRkSymplectic_ = std::shared_ptr< IntegratorSymplecticRk<P_DIM, V_DIM, CONTROL_DIM, SCALAR>>(
            new  IntegratorSymplecticRk<P_DIM, V_DIM, CONTROL_DIM, SCALAR>(
                std::static_pointer_cast< SymplecticSystem<P_DIM, V_DIM, CONTROL_DIM, SCALAR>>(
                    cont_time_system_)));
    }
}

template <typename MNF, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM>
SYMPLECTIC_DISABLED SystemDiscretizer<MNF, CONTROL_DIM, P_DIM, V_DIM>::initializeSymplecticIntegrator()
{
    // leave empty
}

template <typename MNF, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM>
SYMPLECTIC_ENABLED SystemDiscretizer<MNF, CONTROL_DIM, P_DIM, V_DIM>::integrateSymplectic(
     StateVector<STATE_DIM, SCALAR>& x0,
    const double& t,
    const size_t& steps,
    const double& dt_sim) const
{
    if (!cont_time_system_->isSymplectic())
        throw std::runtime_error("Trying to integrate using symplectic integrator, but system is not symplectic.");

    if (integratorType_ ==  IntegrationType::EULER_SYM)
    {
        integratorEulerSymplectic_->integrate_n_steps(x0, t, steps, dt_sim);
    }
    else if (integratorType_ ==  IntegrationType::RK_SYM)
    {
        integratorRkSymplectic_->integrate_n_steps(x0, t, steps, dt_sim);
    }
    else
    {
        throw std::runtime_error("invalid symplectic integrator specified");
    }
}

template <typename MNF, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM>
SYMPLECTIC_DISABLED SystemDiscretizer<MNF, CONTROL_DIM, P_DIM, V_DIM>::integrateSymplectic(
     StateVector<STATE_DIM, SCALAR>& x0,
    const double& t,
    const size_t& steps,
    const double& dt_sim) const
{
    throw std::runtime_error("Symplectic integrator selected but invalid dimensions for it. Check V_DIM>1, P_DIM>1");
}
*/

template <typename MNF, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM>
auto SystemDiscretizer<MNF, CONTROL_DIM, P_DIM, V_DIM>::getSubstates() const -> const StateVectorArrayPtr&
{
    return substepRecorder_->getSubstates();
}

template <typename MNF, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM>
auto SystemDiscretizer<MNF, CONTROL_DIM, P_DIM, V_DIM>::getSubcontrols() const -> const ControlVectorArrayPtr&
{
    return substepRecorder_->getSubcontrols();
}

}  // namespace core
}  // namespace ct

#undef SYMPLECTIC_ENABLED
#undef SYMPLECTIC_DISABLED
