/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/optcon/constraint/LinearConstraintContainer.h>
#include <ct/optcon/costfunction/CostFunctionQuadratic.hpp>

namespace ct {
namespace optcon {


/*!
 * \defgroup OptConProblem OptConProblem
 *
 * \brief Class that defines how to set up an Optimal Control Problem
 *
 * An finite-horizon optimal control problem is generally defined through
 * 	- nonlinear system dynamics
 * 	- cost function (intermediate + terminal cost)
 * 	- initial state
 * 	- box constraints
 * 	- general constraints
 * 	- an overall time horizon
 *
 * 	Note that in most cases, the user can also provide a pointer to the linearized system dynamics. This is optional, and
 * 	in case it is not provided, numerical differentiation will be applied to approximate the linearized dynamics.
 *
 * 	\warning Using numerical differentiation is inefficient and typically slow.
 *
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class OptConProblem
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const size_t STATE_D = STATE_DIM;
    static const size_t CONTROL_D = CONTROL_DIM;

    // typedefs
    typedef ct::core::StateVector<STATE_DIM, SCALAR> state_vector_t;
    typedef std::shared_ptr<core::ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>> DynamicsPtr_t;
    typedef std::shared_ptr<core::LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>> LinearPtr_t;
    typedef std::shared_ptr<optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>> CostFunctionPtr_t;
    typedef std::shared_ptr<optcon::LinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>> ConstraintPtr_t;

    OptConProblem();

    /*!
	 * @brief Construct a simple unconstrained Optimal Control Problem
	 * \warning time and initial state to be specified later
	 *
	 * @param nonlinDynamics the nonlinear system dynamics
	 * @param costFunction a quadratic cost function
	 * @param linearSystem (optional) the linear system holding the dynamics derivatives. If the
	 * user does not specify the derivatives, they are generated automatically using numerical differentiation. Warning: this is slow
	 */
    OptConProblem(DynamicsPtr_t nonlinDynamics, CostFunctionPtr_t costFunction, LinearPtr_t linearSystem = nullptr);

    /*!
     * @brief Construct a simple unconstrained optimal control problem, with initial state and final time as constructor arguments
	 * @param tf The optimal control problem final time horizon
	 * @param x0 The initial system state
	 * @param nonlinDynamics The nonlinear system dynamics
	 * @param costFunction A quadratic cost function
	 * @param linearSystem (optional) Linearized System Dynamics.
	 */
    OptConProblem(const SCALAR& tf,
        const state_vector_t& x0,
        DynamicsPtr_t nonlinDynamics,
        CostFunctionPtr_t costFunction,
        LinearPtr_t linearSystem = nullptr);

    /*!
	 * @brief Construct a constrained Optimal Control Problem
	 *
	 * @param nonlinDynamics the nonlinear system dynamics
	 * @param costFunction a quadratic cost function
	 * @param boxConstraints the box constraints
	 * @param generalConstraints the general constraints
	 * @param linearSystem (optional) the linear system holding the dynamics derivatives.
	 *
	 * \warning time and initial state to be specified later
	 * \warning If the user does not specify the derivatives, they are generated automatically using numerical differentiation. This is slow
	 */
    OptConProblem(DynamicsPtr_t nonlinDynamics,
        CostFunctionPtr_t costFunction,
        ConstraintPtr_t boxConstraints,
        ConstraintPtr_t generalConstraints,
        LinearPtr_t linearSystem = nullptr);

    /*!
	 * @brief Construct a constrained Optimal Control Problem
	 *
	 * @param tf The optimal control problem final time horizon
	 * @param x0 The initial system state
	 * @param nonlinDynamics the nonlinear system dynamics
	 * @param costFunction a quadratic cost function
	 * @param boxConstraints the box constraints
	 * @param generalConstraints the general constraints
	 * @param linearSystem (optional) the linear system holding the dynamics derivatives.
	 *
	 * \warning time and initial state to be specified later
	 * \warning If the user does not specify the derivatives, they are generated automatically using numerical differentiation. This is slow
	 */
    OptConProblem(const SCALAR& tf,
        const state_vector_t& x0,
        DynamicsPtr_t nonlinDynamics,
        CostFunctionPtr_t costFunction,
        ConstraintPtr_t boxConstraints,
        ConstraintPtr_t generalConstraints,
        LinearPtr_t linearSystem = nullptr);

    //! check if all the ingredients for an unconstrained optimal control problem are there
    void verify() const;

    /*!
	 * returns a pointer to the controlled system
	 * */
    const DynamicsPtr_t getNonlinearSystem() const;

    /*!
	 * returns a pointer to the linear system approximation
	 * */
    const LinearPtr_t getLinearSystem() const;

    /*!
	 * returns a pinter to the cost function
	 * */
    const CostFunctionPtr_t getCostFunction() const;

    /*!
	 * returns a pointer to the controlled system
	 * */
    void setNonlinearSystem(const DynamicsPtr_t dyn);

    /*!
	 * returns a pointer to the linear system approximation
	 * */
    void setLinearSystem(const LinearPtr_t lin);

    /*!
	 * returns a pinter to the cost function
	 * */
    void setCostFunction(const CostFunctionPtr_t cost);

    /*!
	 * set box constraints
	 * @param constraint pointer to box constraint
	 */
    void setBoxConstraints(const ConstraintPtr_t constraint);

    /*!
	 * set general constraints
	 * @param constraint pointer to a general constraint
	 */
    void setGeneralConstraints(const ConstraintPtr_t constraint);

    /**
	 * @brief      Retrieve the box constraints
	 *
	 * @return     The box constraints.
	 */
    const ConstraintPtr_t getBoxConstraints() const;

    /**
	 * @brief      Retrieves the general constraints
	 *
	 * @return     The the general constraints
	 */
    const ConstraintPtr_t getGeneralConstraints() const;

    /*!
	 * get initial state (called by solvers)
	 * */
    const state_vector_t getInitialState() const;

    /*!
	 * set initial state for first subsystem
	 * */
    void setInitialState(const state_vector_t& x0);

    /*!
	 * get the current time horizon
	 * @return	Time Horizon
	 */
    const SCALAR& getTimeHorizon() const;

    /*!
	 * Update the current time horizon in the Opt.Control Problem (required for example for replanning)
	 * @param tf new time horizon
	 */
    void setTimeHorizon(const SCALAR& tf);


private:
    SCALAR tf_;  //! end time

    state_vector_t x0_;  //! initial state

    DynamicsPtr_t controlledSystem_;  //! the nonlinear system
    CostFunctionPtr_t costFunction_;  //! a quadratic cost function
    LinearPtr_t linearizedSystem_;    //! the linear approximation of the nonlinear system

    /*!
     * @brief container of all the state and input box constraints of the problem
     * Expected form:
     * \f$ u_{lb} \leq u \leq u_{ub} \f$ and \f$ x_{lb} \leq x \leq x_{ub} \f$
     */
    ConstraintPtr_t boxConstraints_;

    /*!
     * @brief container of all the general constraints of the problem
     * Expected form:
     * \f$ d_{lb} \leq g(x,u) \leq d_{ub} \f$
     */
    ConstraintPtr_t generalConstraints_;
};

}  // namespace optcon
}  // namespace ct
