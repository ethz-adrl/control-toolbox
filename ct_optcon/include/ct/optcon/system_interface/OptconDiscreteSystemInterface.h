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
 * with a discrete optimal control problem.
 *
 * \tparam STATE_DIM size of state vector
 * \tparam CONTROL_DIM size of input vector
 * \tparam SCALAR the underlying scalar type
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class OptconDiscreteSystemInterface : public OptconSystemInterface<STATE_DIM,
                                          CONTROL_DIM,
                                          DiscreteOptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>,
                                          SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef OptconSystemInterface<STATE_DIM, CONTROL_DIM, DiscreteOptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>, SCALAR>
        Base;

    typedef typename Base::control_vector_t control_vector_t;
    typedef typename Base::state_vector_t state_vector_t;
    typedef typename Base::state_matrix_t state_matrix_t;
    typedef typename Base::state_control_matrix_t state_control_matrix_t;

    typedef typename Base::StateVectorArrayPtr StateVectorArrayPtr;
    typedef typename Base::StateSubsteps StateSubsteps;
    typedef typename Base::ControlVectorArrayPtr ControlVectorArrayPtr;
    typedef typename Base::ControlSubsteps ControlSubsteps;

    typedef typename Base::optConProblem_t optConProblem_t;
    typedef typename Base::settings_t settings_t;

    //! constructor
    OptconDiscreteSystemInterface(const optConProblem_t& problem, const settings_t& settings);

    //! destructor
    virtual ~OptconDiscreteSystemInterface() = default;
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
};

}  // namespace optcon
}  // namespace ct
