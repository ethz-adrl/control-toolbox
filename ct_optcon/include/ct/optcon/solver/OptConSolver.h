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


/**
 * OptConSolver.hpp
 *
 * Created on: 28.02.2017
 * 	   Author: mgiftthaler<mgiftthaler@ethz.ch> 
 * 
 *
 * Requirements:
 * - returns an optimal controller. These can be different controller types, feedforward only, feedforward-feedback, feedback only,
 * 		therefore it is templated on the controller type
 */

#ifndef CT_OPTCONSOLVER_HPP_
#define CT_OPTCONSOLVER_HPP_

#include <ct/core/core.h>
#include <ct/optcon/problem/OptConProblem.h>


namespace ct{
namespace optcon{


/** \defgroup OptConSolver OptConSolver
 * Solver interface for finite horizon optimal control problems
 *
 *  * Requirements:
 * - returns an optimal controller. These can be different controller types, feedforward only, feedforward-feedback, feedback only,
 * 		therefore it is templated on the controller type
 */
template <typename DERIVED, typename POLICY, typename SETTINGS, size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class OptConSolver{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	static const size_t STATE_D = STATE_DIM;
	static const size_t CONTROL_D = CONTROL_DIM;

	typedef POLICY Policy_t;
	typedef SETTINGS Settings_t;
	typedef DERIVED Derived;
	typedef SCALAR Scalar_t;

	typedef OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR> OptConProblem_t;


	OptConSolver() {}

	virtual ~OptConSolver(){}

	/*!
	 * \brief Assigns the optimal control problem to the solver.
	 *
	 * Most solvers will require some computational effort to adjust to a new problem.
	 * Therefore, when only adjusting a problem, it is more efficient to adjust individual
	 * properties instead of assigning an entirely new problem. To adjust properties, you can
	 * use
	 *  - changeTimeHorizon()
	 *  - changeInitialState()
	 *  - changeCostFunction()
	 *  - changeSystemDynamics()
	 *  - changeSystemLinearization()
	 *
	 * @return returns true if the optimal control structure is valid for the given solver
	 */
	virtual void setProblem(const OptConProblem_t& optConProblem) {

		optConProblem.verify();

		changeTimeHorizon(optConProblem.getTimeHorizon());
		changeInitialState(optConProblem.getInitialState());
		changeCostFunction(optConProblem.getCostFunction());
		changeNonlinearSystem(optConProblem.getNonlinearSystem());
		changeLinearSystem(optConProblem.getLinearSystem());
		
		if(optConProblem.getStateInputConstraints())
			changeStateInputConstraints(optConProblem.getStateInputConstraints());
		if(optConProblem.getPureStateConstraints())
			changePureStateConstraints(optConProblem.getPureStateConstraints());

	}

	/**
	 * configures the solver
	 * */
	virtual void configure(const Settings_t& settings) = 0;

	/**
	 * configures the solver from configFile
	 * */
	void configureFromFile(const std::string& filename, bool verbose = true, const std::string& ns = DERIVED::SolverName)
	{
		Settings_t settings;
		settings.load(filename, verbose, ns);
		configure(settings);
	}

	/**
	 * solve the optimal control problem
	 * @return true if solve succeeded, false otherwise.
	 * */
	virtual bool solve() = 0;

	/**
	 * run a single iteration of the solver (might not be supported by all solvers)
	 * @return true if a better solution was found
	 */
	virtual bool runIteration() { throw std::runtime_error("runIteration not supported by solver"); }

	/**
	 * Get the optimized control policy to the optimal control problem
	 * @return
	 */
	virtual const Policy_t& getSolution() = 0;

	/**
	 * Get the optimized trajectory to the optimal control problem
	 * @return
	 */
	virtual const core::StateTrajectory<STATE_DIM, SCALAR> getStateTrajectory() const = 0;

	/**
	 * Get the optimal feedforward control input corresponding to the optimal trajectory
	 * @return
	 */
	virtual const core::ControlTrajectory<CONTROL_DIM, SCALAR> getControlTrajectory() const = 0;

	/**
	 * Get the time indices corresponding to the solution
	 * @return
	 */
	virtual const core::tpl::TimeArray<SCALAR>& getTimeArray() const = 0;


	/*!
	 * Set the initial guess used by the solver (not all solvers might support initial guesses)
	 */
	virtual void setInitialGuess(const Policy_t& initialGuess) = 0;


	/*!
	 * \brief Get the time horizon the solver currently operates on.
	 *
	 */
	virtual SCALAR getTimeHorizon() const  = 0;


	/*!
	 * \brief Change the time horizon the solver operates on.
	 *
	 * This function does not need to be called if setOptConProblem() has been called
	 * with an OptConProblem that had the correct time horizon set.
	 */
	virtual void changeTimeHorizon(const SCALAR& tf) = 0;

	/*!
	 * \brief Change the initial state for the optimal control problem
	 *
	 * This function does not need to be called if setOptConProblem() has been called
	 * with an OptConProblem that had the correct initial state set
	 */
	virtual void changeInitialState(const core::StateVector<STATE_DIM, SCALAR>& x0) = 0;

	/*!
	 * \brief Change the cost function
	 *
	 * This function does not need to be called if setOptConProblem() has been called
	 * with an OptConProblem that had the correct cost function
	 */
	virtual void changeCostFunction(const typename OptConProblem_t::CostFunctionPtr_t& cf) = 0;

	/*!
	 * \brief Change the nonlinear system
	 *
	 * This function does not need to be called if setOptConProblem() has been called
	 * with an OptConProblem that had the correct nonlinear system
	 */
	virtual void changeNonlinearSystem(const typename OptConProblem_t::DynamicsPtr_t& dyn) = 0;

	/*!
	 * \brief Change the linear system
	 *
	 * This function does not need to be called if setOptConProblem() has been called
	 * with an OptConProblem that had the correct linear system
	 */
	virtual void changeLinearSystem(const typename OptConProblem_t::LinearPtr_t& lin) = 0;

	/**
	 * @brief      Change the state input constraints
	 *
	 *             This function does not need to be called if
	 *             setOptConProblem() has been called with an OptConProblem that
	 *             had the correct linear system
	 *
	 * @param[in]  con   The new state input constraints
	 */
	virtual void changeStateInputConstraints(const typename OptConProblem_t::ConstraintPtr_t con) 
	{
		throw std::runtime_error("The current solver does not support state input constraints!");
	}

	/**
	 * @brief      Change the pure state constraints.
	 *
	 *             This function does not need to be called if
	 *             setOptConProblem() has been called with an OptConProblem that
	 *             had the correct linear system
	 *
	 * @param[in]  con   The new pure state constraints
	 */
	virtual void changePureStateConstraints(const typename OptConProblem_t::ConstraintPtr_t con)
	{
		throw std::runtime_error("The current solver does not support pure state constraints!");
	}

	virtual SCALAR getCost() const
	{
		throw std::runtime_error("Get cost not implemented");
	}


	/*!
	 * \brief Direct accessor to the system instances
	 *
	 * \warning{Use this only when performance absolutely matters and if you know what you
	 * are doing. Otherwise use e.g. changeNonlinearSystem() to change the system dynamics
	 * in a safe and easy way. You should especially not change the size of the vector or
	 * modify each entry differently.}
	 * @return
	 */
	virtual std::vector<typename OptConProblem_t::DynamicsPtr_t>& getNonlinearSystemsInstances() = 0;

	virtual const std::vector<typename OptConProblem_t::DynamicsPtr_t>& getNonlinearSystemsInstances() const = 0;

	/*!
	 * \brief Direct accessor to the linear system instances
	 *
	 * \warning{Use this only when performance absolutely matters and if you know what you
	 * are doing. Otherwise use e.g. changeLinearSystem() to change the system dynamics
	 * in a safe and easy way. You should especially not change the size of the vector or
	 * modify each entry differently.}
	 * @return
	 */
	virtual std::vector<typename OptConProblem_t::LinearPtr_t>& getLinearSystemsInstances() = 0;

	virtual const std::vector<typename OptConProblem_t::LinearPtr_t>& getLinearSystemsInstances() const = 0;

	/*!
	 * \brief Direct accessor to the cost function instances
	 *
	 * \warning{Use this only when performance absolutely matters and if you know what you
	 * are doing. Otherwise use e.g. changeCostFunction() to change the system dynamics
	 * in a safe and easy way. You should especially not change the size of the vector or
	 * modify each entry differently.}
	 * @return
	 */
	virtual std::vector<typename OptConProblem_t::CostFunctionPtr_t>& getCostFunctionInstances() = 0;

	virtual const std::vector<typename OptConProblem_t::CostFunctionPtr_t>& getCostFunctionInstances() const = 0;

	/**
	 * @brief      Direct accessor to the state input constraint instances
	 * 
	 * \warning{Use this only when performance absolutely matters and if you know what you
	 * are doing. Otherwise use e.g. changeCostFunction() to change the system dynamics
	 * in a safe and easy way. You should especially not change the size of the vector or
	 * modify each entry differently.}
	 *
	 * @return     The state input constraint instances
	 */
	virtual std::vector<typename OptConProblem_t::ConstraintPtr_t>& getStateInputConstraintsInstances() = 0;

	virtual const std::vector<typename OptConProblem_t::ConstraintPtr_t>& getStateInputConstraintsInstances() const = 0;

	/**
	 * @brief      Direct accessor to the pure state constraints
	 * 
	 * \warning{Use this only when performance absolutely matters and if you know what you
	 * are doing. Otherwise use e.g. changeCostFunction() to change the system dynamics
	 * in a safe and easy way. You should especially not change the size of the vector or
	 * modify each entry differently.}
	 *
	 * @return     The pure state constraints instances.
	 */
	virtual std::vector<typename OptConProblem_t::ConstraintPtr_t>& getPureStateConstraintsInstances() = 0;

	virtual const std::vector<typename OptConProblem_t::ConstraintPtr_t>& getPureStateConstraintsInstances() const = 0;


	/**
	 * @brief      Generates and compiles AD source code which can be used in
	 *             the solver. This method can be called just before solving the
	 *             problem, i.e. it can be called from the same executable than
	 *             the actual solve
	 *
	 * @param[in]  problemCG  The optcon problem templated on the AD CG Scalar
	 * @param[in]  settings   The settings indicating what to generate
	 */
	virtual void generateAndCompileCode(
		const OptConProblem<STATE_DIM, CONTROL_DIM, ct::core::ADCGScalar>& problemCG,
		const ct::core::DerivativesCppadSettings& settings)
	{
		throw std::runtime_error("Generate and compile code not implemented for this solver");
	}

	/**
	 * @brief      Generates source AD source code which can be used in the
	 *             solver. This method needs to be called ahead of actually
	 *             solving the problem (e.g. from a different executable)
	 *
	 * @param[in]  settings  The settings indicating what to generate
	 */
	virtual void generateCode(const ct::core::DerivativesCppadSettings& settings) 
	{
		throw std::runtime_error("Generate Code not implemented for this solver");
	}
};


}
}



#endif /* OPTCONSOLVER_HPP_ */
