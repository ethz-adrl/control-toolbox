/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "OptconSystemInterface.h"

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
template <typename MANIFOLD, size_t CONTROL_DIM>
class OptconContinuousSystemInterface
    : public OptconSystemInterface<MANIFOLD, CONTROL_DIM, ct::core::TIME_TYPE::CONTINUOUS_TIME>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Base = OptconSystemInterface<MANIFOLD, CONTROL_DIM, ct::core::TIME_TYPE::CONTINUOUS_TIME>;

    using control_vector_t = typename Base::control_vector_t;
    using state_matrix_t = typename Base::state_matrix_t;
    using state_control_matrix_t = typename Base::state_control_matrix_t;

    using discretizer_t = ct::core::SystemDiscretizer<MANIFOLD, CONTROL_DIM>;
    using system_discretizer_ptr_t = std::shared_ptr<discretizer_t>;

    using Sensitivity_t = ct::core::Sensitivity<MANIFOLD, CONTROL_DIM>;
    using SensitivityPtr = std::shared_ptr<Sensitivity_t>;

    using StateVectorArrayPtr = typename Base::StateVectorArrayPtr;
    using StateSubstepsPtr = typename Base::StateSubstepsPtr;
    using ControlVectorArrayPtr = typename Base::ControlVectorArrayPtr;
    using ControlSubstepsPtr = typename Base::ControlSubstepsPtr;

    using optConProblem_t = typename Base::optConProblem_t;
    using settings_t = typename Base::settings_t;

    OptconContinuousSystemInterface(const optConProblem_t& problem, const settings_t& settings);
    virtual ~OptconContinuousSystemInterface();

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
    virtual void getAandB(const MANIFOLD& x,
        const control_vector_t& u,
        const MANIFOLD& x_next,
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
    virtual void computeControlledDynamics(const MANIFOLD& m,
        const int n,
        const control_vector_t& control,
        typename MANIFOLD::Tangent& t,
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
    //! system discretizers
    std::vector<system_discretizer_ptr_t, Eigen::aligned_allocator<system_discretizer_ptr_t>> discretizers_;

    //! the ct sensitivity integrators
    std::vector<SensitivityPtr, Eigen::aligned_allocator<SensitivityPtr>> sensitivity_;
};

}  // namespace optcon
}  // namespace ct
