/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

//! interface class for optimal control algorithms
/*!
 * Defines the unified interface for optimal control solvers used in conjuction
 * with a continuous optimal control problem.
 * It bundles the optcon problem with system integrators and sensitivities.
 *
 * \tparam STATE_DIM size of state vector
 * \tparam CONTROL_DIM size of input vector
 * \tparam P_DIM size for symplectic integrator
 * \tparam V_DIM size for symplectic integrator
 * \tparam SCALAR the underlying scalar type
 */
template <size_t STATE_DIM,
    size_t CONTROL_DIM,
    size_t P_DIM = STATE_DIM / 2,
    size_t V_DIM = STATE_DIM / 2,
    typename SCALAR = double>
class OptconContinuousSystemInterface : public OptconSystemInterface<STATE_DIM,
                                            CONTROL_DIM,
                                            ContinuousOptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>,
                                            SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef OptconSystemInterface<STATE_DIM,
        CONTROL_DIM,
        ContinuousOptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>,
        SCALAR>
        Base;

    typedef typename Base::control_vector_t control_vector_t;
    typedef typename Base::state_vector_t state_vector_t;
    typedef typename Base::state_matrix_t state_matrix_t;
    typedef typename Base::state_control_matrix_t state_control_matrix_t;

    typedef ct::core::SystemDiscretizer<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR> discretizer_t;
    typedef std::shared_ptr<discretizer_t> system_discretizer_ptr_t;

    typedef ct::core::Sensitivity<STATE_DIM, CONTROL_DIM, SCALAR> Sensitivity_t;
    typedef std::shared_ptr<Sensitivity_t> SensitivityPtr;

    typedef typename Base::StateVectorArrayPtr StateVectorArrayPtr;
    typedef typename Base::StateSubstepsPtr StateSubstepsPtr;
    typedef typename Base::ControlVectorArrayPtr ControlVectorArrayPtr;
    typedef typename Base::ControlSubstepsPtr ControlSubstepsPtr;

    typedef typename Base::optConProblem_t optConProblem_t;
    typedef typename Base::settings_t settings_t;

    //! constructor
    OptconContinuousSystemInterface(const optConProblem_t& problem, const settings_t& settings);

    //! destructor
    virtual ~OptconContinuousSystemInterface() {}
    //! perform necessary setup work
    virtual void initialize() override;
    virtual void configure(const settings_t& settings) override;

    //! retrieve discrete-time linear system matrices A and B.
    /*!
     * @param x	the state setpoint
     * @param u the control setpoint
     * @param x_next the next state
     * @param n the time setpoint
     * @param subSteps number of substeps of trajectory for which to get the sensitivity for
     * @param A the resulting linear system matrix A
     * @param B the resulting linear system matrix B
     * @param threadId which thread specific instantiations to use
     */
    virtual void getAandB(const state_vector_t& x,
        const control_vector_t& u,
        const state_vector_t& x_next,
        const int n,
        size_t subSteps,
        state_matrix_t& A,
        state_control_matrix_t& B,
        const size_t threadId) override;

    //! propagate discrete-time dynamics
    /*!
     * @param state start state to propagate from
     * @param n discrete time index to propagate the dynamics at
     * @param control the control input to apply. This is a constant control input applied during the discretization interval
     * @param stateNext the resulting propagated state
     * @param threadId which thread specific instantiations to use
     */
    virtual void propagateControlledDynamics(const state_vector_t& state,
        const time_t n,
        const control_vector_t& control,
        state_vector_t& stateNext,
        const size_t threadId) override;

    virtual void changeNonlinearSystem(const typename optConProblem_t::DynamicsPtr_t& dyn) override;
    virtual void changeLinearSystem(const typename optConProblem_t::LinearPtr_t& lin) override;

    //! set the number of stages/time steps
    virtual void changeNumStages(const int numStages) override;

    virtual void getSubstates(StateVectorArrayPtr& subStepsX, const size_t threadId) override;
    virtual void getSubcontrols(ControlVectorArrayPtr& subStepsU, const size_t threadId) override;

    virtual void setSubstepTrajectoryReference(const StateSubstepsPtr& xSubsteps,
        const ControlSubstepsPtr& uSubsteps,
        const size_t threadId) override;

private:
    std::vector<system_discretizer_ptr_t, Eigen::aligned_allocator<system_discretizer_ptr_t>>
        discretizers_;  //! system discretizers

    std::vector<SensitivityPtr, Eigen::aligned_allocator<SensitivityPtr>>
        sensitivity_;  //! the ct sensitivity integrators
};

}  // namespace optcon
}  // namespace ct
