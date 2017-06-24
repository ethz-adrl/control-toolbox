/***********************************************************************************
Copyright (c) 2017, Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo,
Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be used
      to endorse or promote products derived from this software without specific
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/


#ifndef CT_OPTIMALCONTROLPROBLEM_H_
#define CT_OPTIMALCONTROLPROBLEM_H_

#include <ct/core/core.h>
#include <ct/optcon/constraint/LinearConstraintContainer.h>
#include <ct/optcon/costfunction/CostFunctionQuadratic.hpp>


namespace ct{
namespace optcon{



/*!
 * \defgroup OptConProblem OptConProblem
 *
 * \brief Class that defines how to set up an Optimal Control Problem
 *
 * An finite-horizon optimal control problem is generally defined through
 * 	- nonlinear system dynamics
 * 	- cost function (intermediate + terminal cost)
 * 	- initial state
 * 	- state-input constraints
 * 	- pure state intermediate constraints
 * 	- pure state terminal constraints
 * 	- an overall time horizon
 *
 * 	Note that in most cases, the user can also provide a pointer to the linearized system dynamics. This is optional, and
 * 	in case it is not provided, numerical differentiation will be applied to approximate the linearized dynamics.
 * 	\warning Using numerical differentiation is inefficient and typically slow.
 *
*/
template<size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class OptConProblem {

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	static const size_t STATE_D = STATE_DIM;
	static const size_t CONTROL_D = CONTROL_DIM;


	// typedefs
	typedef std::shared_ptr<core::ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>> DynamicsPtr_t;
	typedef std::shared_ptr<core::LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>> LinearPtr_t;
	typedef std::shared_ptr<optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>> CostFunctionPtr_t;
	typedef std::shared_ptr<optcon::LinearConstraintContainer<STATE_DIM, CONTROL_DIM>> ConstraintPtr_t;

	OptConProblem(){}


	//! Construct a simple unconstrained Optimal Control Problem
	/*!
	 * Constructor for the simplest problem setup
	 * \warning time and initial state to be specified later
	 *
	 * @param nonlinDynamics the nonlinear system dynamics
	 * @param costFunction a quadratic cost function
	 * @param linearSystem (optional) the linear system holding the dynamics derivatives. If the
	 * user does not specify the derivatives, they are generated automatically using numerical differentiation. Warning: this is slow
	 */
	OptConProblem(
			DynamicsPtr_t nonlinDynamics,
			CostFunctionPtr_t costFunction,
			LinearPtr_t linearSystem = nullptr
	) :
		tf_(0.0),
		x0_(ct::core::StateVector<STATE_DIM, SCALAR>::Zero()),
		controlledSystem_(nonlinDynamics),
		costFunction_(costFunction),
		linearizedSystem_(linearSystem),
		g_(nullptr),
		g_t_(nullptr)
	{
		if(linearSystem == nullptr)	// no linearization provided
		{
			linearizedSystem_ = std::shared_ptr<core::SystemLinearizer<STATE_DIM, CONTROL_DIM, SCALAR>> (
					new core::SystemLinearizer<STATE_DIM, CONTROL_DIM, SCALAR> (controlledSystem_));
		}
	}


	//! Construct a simple unconstrained optimal control problem, with initial state and final time as constructor arguments
	/*!
	 * @param tf The optimal control problem final time horizon
	 * @param x0 The initial system state
	 * @param nonlinDynamics The nonlinear system dynamics
	 * @param costFunction A quadratic cost function
	 * @param linearSystem (optional) Linearized System Dynamics.
	 */
	OptConProblem(
			const SCALAR& tf,
			core::StateVector<STATE_DIM, SCALAR> x0,
			DynamicsPtr_t nonlinDynamics,
			CostFunctionPtr_t costFunction,
			LinearPtr_t linearSystem = nullptr):
				OptConProblem(nonlinDynamics, costFunction, linearSystem)	// delegating constructor
	{
		tf_ = tf;
		x0_ = x0;
	}


	//! check if all the ingredients for an unconstrained otpimal control problem are there
	void verify() const
	{
		if(!controlledSystem_) { throw std::runtime_error("Dynamic system not set"); }
		if(!linearizedSystem_) { throw std::runtime_error("Linearized system not set"); }
		if(!costFunction_) { throw std::runtime_error("Cost function not set"); }
		if(tf_ < 0.0) { throw std::runtime_error("Time horizon should not be negative"); }
	}


	/*!
	 * returns a pointer to the controlled system
	 * */
	const DynamicsPtr_t getNonlinearSystem() const { return controlledSystem_; }


	/*!
	 * returns a pointer to the linear system approximation
	 * */
	const LinearPtr_t getLinearSystem() const { return linearizedSystem_; }


	/*!
	 * returns a pinter to the cost function
	 * */
	const CostFunctionPtr_t getCostFunction() const { return costFunction_; }


	/*!
	 * returns a pointer to the controlled system
	 * */
	void setNonlinearSystem(const DynamicsPtr_t dyn) { controlledSystem_ = dyn; }


	/*!
	 * returns a pointer to the linear system approximation
	 * */
	void setLinearSystem(const LinearPtr_t lin) { linearizedSystem_ = lin; }


	/*!
	 * returns a pinter to the cost function
	 * */
	void setCostFunction(const CostFunctionPtr_t cost) { costFunction_ = cost; }

	/*!
	 * set intermediate constraints
	 * @param constraint pointer to intermediate constraint
	 */
	void setIntermediateConstraints(const ConstraintPtr_t constraint) { g_ = constraint;}

	/*!
	 * set final constraints
	 * @param constraint pointer to a final constraint
	 */
	void setFinalConstraints(const ConstraintPtr_t constraint) { g_t_ = constraint;}


	//! retrieve intermediate constraints
	const ConstraintPtr_t getIntermediateConstraints() const { return g_; }

	//! retrieve final constraints
	const ConstraintPtr_t getFinalConstraints() const { return g_t_; }

	/*!
	 * get initial state (called by solvers)
	 * */
	const core::StateVector<STATE_DIM, SCALAR> getInitialState() const {return x0_;}

	/*!
	 * set initial state for first subsystem
	 * */
	void setInitialState(const core::StateVector<STATE_DIM, SCALAR> x0) {x0_ = x0;}


	/*!
	 * get the current time horizon
	 * @return	Time Horizon
	 */
	const SCALAR& getTimeHorizon() const {return tf_ ;}

	/*!
	 * Update the current time horizon in the Opt.Control Problem (required for example for replanning)
	 * @param tf new time horizon
	 */
	void setTimeHorizon(const SCALAR& tf){tf_ = tf;}



private:
	SCALAR tf_;						//! end time

	ct::core::StateVector<STATE_DIM, SCALAR> x0_;	//! initial state

	DynamicsPtr_t controlledSystem_;	//! the nonlinear system
	CostFunctionPtr_t costFunction_;	//! a quadratic cost function
	LinearPtr_t linearizedSystem_;		//! the linear approximation of the nonlinear system

	ConstraintPtr_t g_;		//! container of all the intermediate constraints of the problem
	ConstraintPtr_t g_t_;	//! container of all the terminal constraints of the problem

};


}
}


#endif /* OPTIMALCONTROLPROBLEM_H_ */
