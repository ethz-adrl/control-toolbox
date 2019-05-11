/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "DiscreteControlledSystem.h"
#include "../continuous_time/ControlledSystem.h"
#include <ct/core/control/continuous_time/ConstantController.h>

#include <ct/core/integration/Integrator.h>
#include <ct/core/integration/IntegratorSymplectic.h>
#include <ct/core/integration/EventHandlers/SubstepRecorder.h>

#define SYMPLECTIC_ENABLED                   \
    template <size_t V, size_t P, size_t ST> \
    typename std::enable_if<(V > 0 && P > 0 && (V + P == ST)), void>::type
#define SYMPLECTIC_DISABLED                  \
    template <size_t V, size_t P, size_t ST> \
    typename std::enable_if<(V <= 0 || P <= 0 || (V + P != ST)), void>::type


namespace ct {
namespace core {

//! Discretize a general, continuous-time non-linear dynamic system using forward integration
/*!
 * The SystemDiscretizer transforms a continuous-time system into a discrete-time system by forward integration.
 * Please not that it does not perform a global transformation to discrete-time (it cannot compute transition matrices
 * or similar), but rather 'mimicks' a discrete system, based on the underlying continuous-time system.
 * In every call to propagateControlledDynamics(), the continuous-time system is forward integrated by a time interval
 * dt_. Furthermore, the substeps during integration are recorded during each propagate-call, and can be retrieved
 * using getSubstates() and getSubcontrols().
 *
 *  \warning no substeps can be recorded for higher order RK symplectic integrators.
 *
 */
template <size_t STATE_DIM,
    size_t CONTROL_DIM,
    size_t P_DIM = STATE_DIM / 2,
    size_t V_DIM = STATE_DIM / 2,
    typename SCALAR = double>
class SystemDiscretizer : public DiscreteControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    // convenience typedefs
    typedef DiscreteControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR> Base;
    typedef typename Base::time_t time_t;

    using IntegratorPtr = std::shared_ptr<Integrator<STATE_DIM, SCALAR>>;
    using IntegratorSymplecticEulerPtr =
        std::shared_ptr<ct::core::IntegratorSymplecticEuler<P_DIM, V_DIM, CONTROL_DIM, SCALAR>>;
    using IntegratorSymplecticRkPtr =
        std::shared_ptr<ct::core::IntegratorSymplecticRk<P_DIM, V_DIM, CONTROL_DIM, SCALAR>>;

    using ContinuousSystemPtr = std::shared_ptr<ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>>;
    using ContinuousConstantControllerPtr = std::shared_ptr<ConstantController<STATE_DIM, CONTROL_DIM, SCALAR>>;

    using SubstepRecorderPtr = std::shared_ptr<ct::core::SubstepRecorder<STATE_DIM, CONTROL_DIM, SCALAR>>;

    using StateVectorArray = ct::core::StateVectorArray<STATE_DIM, SCALAR>;
    using StateVectorArrayPtr = std::shared_ptr<StateVectorArray>;
    using ControlVectorArray = ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>;
    using ControlVectorArrayPtr = std::shared_ptr<ControlVectorArray>;

    //! default constructor
    SystemDiscretizer();

    //! constructor with parameters
    /*!
     * @param dt the discretization time interval
     * @param integratorType the integratorType for numerical forward integration
     * @param K_sim the number of sub-integration intervals
     */
    SystemDiscretizer(const SCALAR& dt,
        const ct::core::IntegrationType& integratorType = ct::core::IntegrationType::RK4,
        const int& K_sim = 1);

    //! constructor with parameters and nonlinear continuous-time system
    /*!
     *
     * @param system The continuous-time system to be discretized
     * @param dt the discretization time interval
     * @param integratorType the integratorType for numerical forward integration
     * @param K_sim the number of sub-integration intervals
     */
    SystemDiscretizer(ContinuousSystemPtr system,
        const SCALAR& dt,
        const ct::core::IntegrationType& integratorType = ct::core::IntegrationType::RK4,
        const int& K_sim = 1);

    //! copy constructor
    SystemDiscretizer(const SystemDiscretizer& arg);

    //! destructor
    virtual ~SystemDiscretizer();

    //! deep cloning
    virtual SystemDiscretizer<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>* clone() const override;

    //! initialize class
    void initialize();

    //! update integration type
    void setIntegrationType(const ct::core::IntegrationType& integratorType);

    //! update parameters
    void setParameters(const SCALAR& dt, const int& K_sim = 1);

    //! update the SystemDiscretizer with a new nonlinear, continuous-time system
    void changeContinuousTimeSystem(ContinuousSystemPtr newSystem);

    //! propagate discrete-time dynamics by performing a numerical forward integration of the continuous-time system
    /*!
	 * @param state start state to propagate from
	 * @param control the control input to apply. This is a constant control input applied to the continuous-time dynamics
	 * @param n time index to propagate the dynamics at, to be translated into a continuous time value t
	 * @param stateNext the resulting propagated state
	 *
	 * \warning calling this method resets the substep-recorder. The substeps are only available for a single call to propagateControlledDynamics()
	 */
    virtual void propagateControlledDynamics(const StateVector<STATE_DIM, SCALAR>& state,
        const time_t n,
        const ControlVector<CONTROL_DIM, SCALAR>& control,
        StateVector<STATE_DIM, SCALAR>& stateNext) override;

    //! return a pointer to the substates recorded during integration
    const StateVectorArrayPtr& getSubstates() const;

    //! reuturn a pointer to the subcontrols recorded during integration
    const ControlVectorArrayPtr& getSubcontrols() const;

protected:
    //! initialize the symplectic integrator, if the system is symplectic
    SYMPLECTIC_ENABLED initializeSymplecticIntegrator();

    //! a dummy method, instantiated if the system is not symplectic
    SYMPLECTIC_DISABLED initializeSymplecticIntegrator();

    //! integrateSymplectic, gets instantiated if the system is symplectic
    SYMPLECTIC_ENABLED integrateSymplectic(ct::core::StateVector<STATE_DIM, SCALAR>& x0,
        const double& t,
        const size_t& steps,
        const double& dt_sim) const;

    //! a dummy method, instantiated if the system is not symplectic
    SYMPLECTIC_DISABLED integrateSymplectic(ct::core::StateVector<STATE_DIM, SCALAR>& x0,
        const double& t,
        const size_t& steps,
        const double& dt_sim) const;

    //! compute the simulation timestep
    SCALAR getSimulationTimestep();

    //! the time discretization interval
    SCALAR dt_;

    //! the forward simulation can be discretized finer than dt_. K_sim_ is an integer number representing the number of simulation sub-steps.
    int K_sim_;

    //! the integration sub-step size, which is a function of dt_ and K_sim_
    SCALAR dt_sim_;

    //! the integration type for forward integration
    ct::core::IntegrationType integratorType_;

    //! the continuous-time system to be discretized
    ContinuousSystemPtr cont_time_system_;

    //! the continuous-time constant controller to be applied to the continuous-time dynamics
    ContinuousConstantControllerPtr cont_constant_controller_;

    //! an integrator for forward integrating a general continuous-time system with standard RK methods
    IntegratorPtr integrator_;

    //! an integrator for forward integrating a symplectic continuous-time system with symplectic Euler
    IntegratorSymplecticEulerPtr integratorEulerSymplectic_;

    //! an integrator for forward integrating a symplectic continuous-time system with symplectic RK methods
    IntegratorSymplecticRkPtr integratorRkSymplectic_;

    //! substep recorder which logs all the substeps required for later computing exact sensitivities
    SubstepRecorderPtr substepRecorder_;
};


}  // namespace core
}  // namespace ct

#undef SYMPLECTIC_ENABLED
#undef SYMPLECTIC_DISABLED
