/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/optcon/solver/NLOptConSettings.hpp>

namespace ct {
namespace optcon {

//! interface base class for optimal control algorithms
/*!
 * Defines the unified interface for optimal control solvers.
 * It bundles the optimal control problem with required tools to propagate
 * the system dynamics and provide corresponding sensitivities.
 *
 * \tparam STATE_DIM size of state vector
 * \tparam CONTROL_DIM size of input vector
 * \tparam OPTCONPROBLEM type of the optConProblem (continuous or discrete)
 * \tparam SCALAR the underlying scalar type
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename OPTCONPROBLEM, typename SCALAR = double>
class OptconSystemInterface
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef ct::core::ControlVector<CONTROL_DIM, SCALAR> control_vector_t;
    typedef ct::core::StateVector<STATE_DIM, SCALAR> state_vector_t;
    typedef ct::core::StateMatrix<STATE_DIM, SCALAR> state_matrix_t;
    typedef ct::core::StateControlMatrix<STATE_DIM, CONTROL_DIM, SCALAR> state_control_matrix_t;

    typedef ct::core::StateVectorArray<STATE_DIM, SCALAR> StateVectorArray;
    typedef std::shared_ptr<StateVectorArray> StateVectorArrayPtr;
    typedef std::vector<StateVectorArrayPtr, Eigen::aligned_allocator<StateVectorArrayPtr>> StateSubsteps;
    typedef std::shared_ptr<StateSubsteps> StateSubstepsPtr;

    typedef ct::core::ControlVectorArray<CONTROL_DIM, SCALAR> ControlVectorArray;
    typedef std::shared_ptr<ControlVectorArray> ControlVectorArrayPtr;
    typedef std::vector<ControlVectorArrayPtr, Eigen::aligned_allocator<ControlVectorArrayPtr>> ControlSubsteps;
    typedef std::shared_ptr<ControlSubsteps> ControlSubstepsPtr;

    typedef ct::core::ConstantController<STATE_DIM, CONTROL_DIM, SCALAR> constant_controller_t;
    typedef std::shared_ptr<constant_controller_t> ConstantControllerPtr;

    typedef OPTCONPROBLEM optConProblem_t;
    typedef NLOptConSettings settings_t;

    //! constructor
    OptconSystemInterface(const optConProblem_t& problem, const settings_t& settings)
        : controller_(settings.nThreads + 1), optConProblem_(problem), settings_(settings)
    {
        for (auto& controller_i : controller_)
        {
            controller_i = ConstantControllerPtr(new constant_controller_t());
        }
    }

    //! perform any required setup work
    virtual void initialize() {}
    virtual void configure(const settings_t& settings) {}
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
        const size_t threadId) = 0;


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
        const size_t threadId) = 0;

    //! set the number of stages/time steps
    virtual void changeNumStages(const int numStages) {}
    const optConProblem_t& getOptConProblem() { return optConProblem_; };
    std::vector<typename optConProblem_t::DynamicsPtr_t>& getNonlinearSystemsInstances() { return systems_; }
    std::vector<typename optConProblem_t::LinearPtr_t>& getLinearSystemsInstances() { return linearSystems_; }
    virtual void changeNonlinearSystem(const typename optConProblem_t::DynamicsPtr_t& dyn) = 0;
    virtual void changeLinearSystem(const typename optConProblem_t::LinearPtr_t& lin) = 0;

    virtual void getSubstates(StateVectorArrayPtr& subStepsX, const size_t threadId) {}
    virtual void getSubcontrols(ControlVectorArrayPtr& subStepsU, const size_t threadId) {}
    virtual void setSubstepTrajectoryReference(const StateSubstepsPtr& xSubsteps,
        const ControlSubstepsPtr& uSubsteps,
        const size_t threadId){};

protected:
    /*!
     * of the following objects, we have nThreads+1 instantiations in form of a vector.
     * Every instantiation is dedicated to a certain thread in the multi-thread implementation
     */
    std::vector<typename optConProblem_t::DynamicsPtr_t> systems_;
    std::vector<typename optConProblem_t::LinearPtr_t> linearSystems_;


    std::vector<ConstantControllerPtr, Eigen::aligned_allocator<ConstantControllerPtr>>
        controller_;  //! the constant controller for forward-integration during one time-step

    optConProblem_t optConProblem_;  //! instance of the optconProblem

    settings_t settings_;  //! instance of the optcon Settings
};

}  // namespace optcon
}  // namespace ct
