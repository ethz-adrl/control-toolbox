/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/systems/ControlledSystem.h>
#include <ct/core/control/ConstantController.h>

#include <ct/core/integration/Integrator.h>
//#include <ct/core/integration/IntegratorSymplectic.h> // TODO: bring back?
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

// TODO: bring back symplectic stuff, which is currently commented out
template <typename MNF, size_t CONTROL_DIM, size_t P_DIM = MNF::TangentDim / 2, size_t V_DIM = MNF::TangentDim / 2>
class SystemDiscretizer : public ControlledSystem<MNF, CONTROL_DIM, DISCRETE_TIME>
{
public:
    using SCALAR = typename MNF::Scalar;
    using Base = ControlledSystem<MNF, CONTROL_DIM, DISCRETE_TIME>;
    using Time_t = typename Base::Time_t;
    static constexpr size_t STATE_DIM = MNF::TangentDim;

    using IntegratorPtr = std::shared_ptr<Integrator<MNF>>;
    //using IntegratorSymplecticEulerPtr = std::shared_ptr< IntegratorSymplecticEuler<P_DIM, V_DIM, CONTROL_DIM, SCALAR>>;
    //using IntegratorSymplecticRkPtr = std::shared_ptr< IntegratorSymplecticRk<P_DIM, V_DIM, CONTROL_DIM, SCALAR>>;

    using ContinuousControlledSystem_t = ControlledSystem<MNF, CONTROL_DIM, CONTINUOUS_TIME>;
    using ContinuousSystemPtr = std::shared_ptr<ContinuousControlledSystem_t>;
    using ContinuousConstantControllerPtr = std::shared_ptr<ConstantController<MNF, CONTROL_DIM, CONTINUOUS_TIME>>;
    using SubstepRecorderPtr = std::shared_ptr<SubstepRecorder<MNF, CONTROL_DIM>>;

    using StateVectorArrayPtr = std::shared_ptr<DiscreteArray<MNF>>;
    using ControlVectorArrayPtr = std::shared_ptr<ControlVectorArray<CONTROL_DIM, SCALAR>>;

    //! default constructor
    SystemDiscretizer();

    //! constructor with parameters
    /*!
     * @param dt the discretization time interval
     * @param integratorType the integratorType for numerical forward integration
     * @param K_sim the number of sub-integration intervals
     */
    SystemDiscretizer(const SCALAR& dt,
        const IntegrationType& integratorType = IntegrationType::RK4,
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
        const IntegrationType& integratorType = IntegrationType::RK4,
        const int& K_sim = 1);

    //! copy constructor
    SystemDiscretizer(const SystemDiscretizer& arg);

    //! destructor
    virtual ~SystemDiscretizer();

    //! deep cloning
    virtual SystemDiscretizer<MNF, CONTROL_DIM, P_DIM, V_DIM>* clone() const override;

    //! initialize class
    void initialize();

    //! update integration type
    void setIntegrationType(const IntegrationType& integratorType);

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
    virtual void computeControlledDynamics(const MNF& m,
        const Time_t& tn,
        const ControlVector<CONTROL_DIM, SCALAR>& control,
        typename MNF::Tangent& t) override;

    //! return a pointer to the substates recorded during integration
    const StateVectorArrayPtr& getSubstates() const;

    //! reuturn a pointer to the subcontrols recorded during integration
    const ControlVectorArrayPtr& getSubcontrols() const;

protected:
    //  //! initialize the symplectic integrator, if the system is symplectic
    //  SYMPLECTIC_ENABLED initializeSymplecticIntegrator();
    //
    //  //! a dummy method, instantiated if the system is not symplectic
    //  SYMPLECTIC_DISABLED initializeSymplecticIntegrator();
    //
    //  //! integrateSymplectic, gets instantiated if the system is symplectic
    //  SYMPLECTIC_ENABLED integrateSymplectic( StateVector<STATE_DIM, SCALAR>& x0,
    //      const double& t,
    //      const size_t& steps,
    //      const double& dt_sim) const;
    //
    //  //! a dummy method, instantiated if the system is not symplectic
    //  SYMPLECTIC_DISABLED integrateSymplectic( StateVector<STATE_DIM, SCALAR>& x0,
    //      const double& t,
    //      const size_t& steps,
    //      const double& dt_sim) const;

    //! compute the simulation timestep
    SCALAR getSimulationTimestep();

    //! the time discretization interval
    SCALAR dt_;

    //! the forward simulation can be discretized finer than dt_. K_sim_ is an integer number representing the number of simulation sub-steps.
    int K_sim_;

    //! the integration sub-step size, which is a function of dt_ and K_sim_
    SCALAR dt_sim_;

    //! the integration type for forward integration
    IntegrationType integratorType_;

    //! the continuous-time system to be discretized
    ContinuousSystemPtr cont_time_system_;

    //! the continuous-time constant controller to be applied to the continuous-time dynamics
    ContinuousConstantControllerPtr cont_constant_controller_;

    //! an integrator for forward integrating a general continuous-time system with standard RK methods
    IntegratorPtr integrator_;

    //! an integrator for forward integrating a symplectic continuous-time system with symplectic Euler
    //IntegratorSymplecticEulerPtr integratorEulerSymplectic_;

    //! an integrator for forward integrating a symplectic continuous-time system with symplectic RK methods
    //IntegratorSymplecticRkPtr integratorRkSymplectic_;

    //! substep recorder which logs all the substeps required for later computing exact sensitivities
    SubstepRecorderPtr substepRecorder_;
};


}  // namespace core
}  // namespace ct

#undef SYMPLECTIC_ENABLED
#undef SYMPLECTIC_DISABLED
