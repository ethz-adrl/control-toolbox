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


#ifndef INCLUDE_CT_OPTCON_NLOC_BACKEND_BASE_HPP_
#define INCLUDE_CT_OPTCON_NLOC_BACKEND_BASE_HPP_

#include <atomic>

#include <ct/core/core.h>
#include <ct/optcon/costfunction/CostFunctionQuadratic.hpp>
#include <ct/optcon/solver/OptConSolver.h>

#include <ct/optcon/problem/LQOCProblem.hpp>
#include <ct/optcon/solver/lqp/GNRiccatiSolver.hpp>
#include <ct/optcon/solver/lqp/HPIPMInterface.hpp>
#include <ct/optcon/solver/NLOptConSettings.hpp>

#ifdef MATLAB
#include <ct/optcon/matlab.hpp>
#endif

namespace ct{
namespace optcon{


/*!
 * \ingroup GNMS
 *
 * \brief C++ implementation of GNMS.
 *
 *  The implementation and naming is based on the reference below. In general, the code follows this convention:
 *  X  <- Matrix (upper-case in paper)
 *  xv <- vector (lower-case bold in paper)
 *  x  <- scalar (lower-case in paper)
 */

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR = double>
class NLOCBackendBase
{

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	static const size_t state_dim = STATE_DIM;
	static const size_t control_dim = CONTROL_DIM;

	typedef NLOptConSettings Settings_t;
	typedef core::StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR> Policy_t;

	typedef OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR> OptConProblem_t;

	typedef OptConSolver<NLOCBackendBase, Policy_t, Settings_t, STATE_DIM, CONTROL_DIM, SCALAR> Base;

	typedef LQOCProblem<STATE_DIM, CONTROL_DIM, SCALAR> LQOCProblem_t;
	typedef LQOCSolver<STATE_DIM, CONTROL_DIM, SCALAR>  LQOCSolver_t;

	typedef core::LinearSystemDiscretizer<STATE_DIM, CONTROL_DIM, SCALAR> LinearSystemDiscretizer_t;

	typedef ct::core::StateVectorArray<STATE_DIM, SCALAR> StateVectorArray;
	typedef ct::core::ControlVectorArray<CONTROL_DIM, SCALAR> ControlVectorArray;

	typedef ct::core::ControlMatrix<CONTROL_DIM, SCALAR> ControlMatrix;
	typedef ct::core::ControlMatrixArray<CONTROL_DIM, SCALAR> ControlMatrixArray;
	typedef ct::core::StateMatrixArray<STATE_DIM, SCALAR> StateMatrixArray;
	typedef ct::core::StateControlMatrixArray<STATE_DIM, CONTROL_DIM, SCALAR> StateControlMatrixArray;
	typedef ct::core::FeedbackArray<STATE_DIM, CONTROL_DIM, SCALAR> FeedbackArray;
	typedef ct::core::tpl::TimeArray<SCALAR> TimeArray;

	typedef Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM> state_matrix_t;
	typedef Eigen::Matrix<SCALAR, CONTROL_DIM, CONTROL_DIM> control_matrix_t;
	typedef Eigen::Matrix<SCALAR, CONTROL_DIM, STATE_DIM> control_state_matrix_t;
	typedef Eigen::Matrix<SCALAR, STATE_DIM, CONTROL_DIM> state_control_matrix_t;

	typedef core::StateVector<STATE_DIM, SCALAR> state_vector_t;
	typedef core::ControlVector<CONTROL_DIM, SCALAR> control_vector_t;


	typedef SCALAR scalar_t;
	typedef std::vector<SCALAR, Eigen::aligned_allocator<SCALAR>> scalar_array_t;



	NLOCBackendBase(const OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>& optConProblem,
			const Settings_t& settings) :

		    integratorsRK4_(settings.nThreads+1),
		    integratorsEuler_(settings.nThreads+1),
			integratorsEulerSymplectic_(settings.nThreads+1),
			integratorsRkSymplectic_(settings.nThreads+1),
			linearSystemDiscretizers_(settings.nThreads+1, LinearSystemDiscretizer_t(settings.dt)),

		    controller_(settings.nThreads+1),
		    settings_(settings),
		    lqocProblem_(new LQOCProblem<STATE_DIM, CONTROL_DIM, SCALAR>())

	{
		for (size_t i=0; i<settings.nThreads+1; i++)
		{
			controller_[i] = ConstantStateFeedbackControllerPtr (new core::ConstantStateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>());
		}

		configure(settings);

		changeTimeHorizon(optConProblem.getTimeHorizon());
		changeInitialState(optConProblem.getInitialState());
		changeCostFunction(optConProblem.getCostFunction());
		changeNonlinearSystem(optConProblem.getNonlinearSystem());
		changeLinearSystem(optConProblem.getLinearSystem());

		// to be done
//		if(optConProblem.getStateInputConstraints())
//			changeStateInputConstraints(optConProblem.getStateInputConstraints());
//		if(optConProblem.getPureStateConstraints())
//			changePureStateConstraints(optConProblem.getPureStateConstraints());
	}


	NLOCBackendBase(const OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>& optConProblem,
			 const std::string& settingsFile,
			 bool verbose = true,
			 const std::string& ns = "alg") :
			NLOCBackendBase(optConProblem, Settings_t::fromConfigFile(settingsFile, verbose, ns))
	{}


	virtual ~NLOCBackendBase() {};


	//! configure the solver
	/**
	 * Configure the solver
	 * @param settings solver settings
	 */
	virtual void configure(const Settings_t& settings);


	//! get the current SLQsolver settings
	const Settings_t& getSettings() const { return settings_; }


	/*!
	 * Set the initial guess used by the solver (not all solvers might support initial guesses)
	 */
	void setInitialGuess(const Policy_t& initialGuess);


	/*!
	 * \brief Change the time horizon the solver operates on.
	 *
	 * This function does not need to be called if setOptConProblem() has been called
	 * with an OptConProblem that had the correct time horizon set.
	 */
	void changeTimeHorizon(const SCALAR& tf);


	SCALAR getTimeHorizon() {return K_* settings_.dt ;}

	int getNumSteps() {return K_;}


	/*!
	 * \brief Change the initial state for the optimal control problem
	 *
	 * This function does not need to be called if setOptConProblem() has been called
	 * with an OptConProblem that had the correct initial state set
	 */
	void changeInitialState(const core::StateVector<STATE_DIM, SCALAR>& x0);

	/*!
	 * \brief Change the cost function
	 *
	 * This function does not need to be called if setOptConProblem() has been called
	 * with an OptConProblem that had the correct cost function
	 */
	void changeCostFunction(const typename Base::OptConProblem_t::CostFunctionPtr_t& cf);

	/*!
	 * \brief Change the nonlinear system
	 *
	 * This function does not need to be called if setOptConProblem() has been called
	 * with an OptConProblem that had the correct nonlinear system
	 */
	void changeNonlinearSystem(const typename Base::OptConProblem_t::DynamicsPtr_t& dyn);

	/*!
	 * \brief Change the linear system
	 *
	 * This function does not need to be called if setOptConProblem() has been called
	 * with an OptConProblem that had the correct linear system
	 */
	void changeLinearSystem(const typename Base::OptConProblem_t::LinearPtr_t& lin);


	/*!
	 * \brief Direct accessor to the system instances
	 *
	 * \warning{Use this only when performance absolutely matters and if you know what you
	 * are doing. Otherwise use e.g. changeNonlinearSystem() to change the system dynamics
	 * in a safe and easy way. You should especially not change the size of the vector or
	 * modify each entry differently.}
	 * @return
	 */
	std::vector<typename OptConProblem_t::DynamicsPtr_t>& getNonlinearSystemsInstances() { return systems_; }

	const std::vector<typename OptConProblem_t::DynamicsPtr_t>& getNonlinearSystemsInstances() const { return systems_; }

	/*!
	 * \brief Direct accessor to the linear system instances
	 *
	 * \warning{Use this only when performance absolutely matters and if you know what you
	 * are doing. Otherwise use e.g. changeLinearSystem() to change the system dynamics
	 * in a safe and easy way. You should especially not change the size of the vector or
	 * modify each entry differently.}
	 * @return
	 */
	std::vector<typename OptConProblem_t::LinearPtr_t>& getLinearSystemsInstances() { return linearSystems_; }

	const std::vector<typename OptConProblem_t::LinearPtr_t>& getLinearSystemsInstances() const { return linearSystems_; }

	/*!
	 * \brief Direct accessor to the cost function instances
	 *
	 * \warning{Use this only when performance absolutely matters and if you know what you
	 * are doing. Otherwise use e.g. changeCostFunction() to change the system dynamics
	 * in a safe and easy way. You should especially not change the size of the vector or
	 * modify each entry differently.}
	 * @return
	 */
	std::vector<typename OptConProblem_t::CostFunctionPtr_t>& getCostFunctionInstances() { return costFunctions_; }

	const std::vector<typename OptConProblem_t::CostFunctionPtr_t>& getCostFunctionInstances() const { return costFunctions_; }

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
	std::vector<typename OptConProblem_t::ConstraintPtr_t>& getStateInputConstraintsInstances() { return stateInputConstraints_; }

	const std::vector<typename OptConProblem_t::ConstraintPtr_t>& getStateInputConstraintsInstances() const { return stateInputConstraints_; }

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
	std::vector<typename OptConProblem_t::ConstraintPtr_t>& getPureStateConstraintsInstances() { return pureStateConstraints_; }

	const std::vector<typename OptConProblem_t::ConstraintPtr_t>& getPureStateConstraintsInstances() const { return pureStateConstraints_; }


	/*!
	 * Tests consistency of the instance of the dynamics, linear systems and costs. This is not a test for thread safety.
	 * @return returns true if instances are consistent with each other
	 */
	bool testConsistency()
	{
		return true;
	}


	//! Export all functions to matlab workspace
	/*!
	  This function can be used for Debugging. It exports all variables to Matlab after each iteration. It also saves
	  the Matlab workspace to a .mat file.
	*/
	void logToMatlab(const size_t& iteration);

	void logInitToMatlab();

	SCALAR getCost() const;

	SCALAR getTotalDefect() const { return d_norm_;}

	void reset()
	{
		iteration_ = 0;
		d_norm_ = std::numeric_limits<scalar_t>::infinity();
		lx_norm_ = std::numeric_limits<scalar_t>::infinity();
		lu_norm_ = std::numeric_limits<scalar_t>::infinity();
		intermediateCostBest_ = std::numeric_limits<scalar_t>::infinity();
		finalCostBest_ = std::numeric_limits<scalar_t>::infinity();
		intermediateCostPrevious_ = std::numeric_limits<scalar_t>::infinity();
		finalCostPrevious_ = std::numeric_limits<scalar_t>::infinity();
	}

	const core::StateTrajectory<STATE_DIM, SCALAR> getStateTrajectory() const;

	const core::ControlTrajectory<CONTROL_DIM, SCALAR> getControlTrajectory() const;

	const Policy_t& getSolution() { return policy_; }

	const TimeArray& getTimeArray() {return t_;}

	bool isConfigured() {return configured_;}

	bool isInitialized() {return initialized_;}


	/*!
	 * the prepare Solve LQP Problem method is intended for a special use-case: unconstrained GNMS with pre-solving of the
	 */
	virtual void prepareSolveLQProblem();


	virtual void finishSolveLQProblem();

	/*!
	 * solve Full LQProblem, e.g. to be used with HPIPM or if we have a constrained problem
	 */
	virtual void solveFullLQProblem();

	//! compute costs of solution candidate
	void updateCosts();

	//! check if GNMS is converged
	bool isConverged();

	//! nominal rollout using default thread and member variables for the results.
	bool nominalRollout() { return rolloutSystem(settings_.nThreads, u_ff_, x_, u_ff_, t_); }

	//! check problem for consistency
	void checkProblem();

	size_t& iteration() {return iteration_;}


	//! Print debugging information
	/*!
	 *  This function is automatically called if the DEBUG_PRINT compileflag is set. It prints out important information
	 *  like cost etc. after each iteration.
	 */
	void debugPrint();

	//! perform line-search and update controller
	bool lineSearchController();

	//! Computes the linearization of the dynamics along the trajectory, for the specified indices. See computeLinearizedDynamics for details
	virtual void computeLinearizedDynamicsAroundTrajectory(size_t firstIndex, size_t lastIndex) = 0;

	//! Computes the quadratic approximation of the cost function along the trajectory, for the specified indices
	virtual void computeQuadraticCostsAroundTrajectory(size_t firstIndex, size_t lastIndex) = 0;

	virtual void updateSolutionState() = 0;

	virtual void updateSolutionFeedforward() = 0;

	virtual void updateSolutionFeedback() = 0;

	//! integrates the specified shots and computes the corresponding defects
	virtual void rolloutShots(size_t firstIndex, size_t lastIndex) = 0;

	virtual SCALAR performLineSearch() = 0;


protected:

	//! integrate the individual shots
	void rolloutSingleShot(size_t threadId, size_t k);

	//! computes the defect between shot and trajectory
	void computeSingleDefect(size_t threadId, size_t k);


    //! Rollout of nonlinear dynamics
    /*!
      This rolls out the nonlinear dynamics to obtain the reference trajectory
    */
	bool rolloutSystem(
			size_t threadId,
			const ControlVectorArray& u_ff_local,
			ct::core::StateVectorArray<STATE_DIM, SCALAR>& x_local,
			ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>& u_local,
			ct::core::tpl::TimeArray<SCALAR>& t_local,
			std::atomic_bool* terminationFlag = nullptr) const;


	//! Computes the linearized Dynamics at a specific point of the trajectory
	/*!
	  This function calculates the linearization, i.e. matrices A and B in \f$ \dot{x} = A(x(k)) x + B(x(k)) u \f$
	  at a specific point of the trajectory

	  \param threadId the id of the worker thread
	  \param k step k
	*/
	void computeLinearizedDynamics(size_t threadId, size_t k);

	//! Computes the quadratic costs
	/*!
	  This function calculates the quadratic costs as provided by the costFunction pointer.

	 * \param threadId id of worker thread
	 * \param k step k
	*/
	void computeQuadraticCosts(size_t threadId, size_t k);

	//! Initializes cost to go
	/*!
	 * This function initializes the cost-to-go function at time K.
     *
	 */
	void initializeCostToGo();

	//! Computes cost to go
	/*!
	 * This function computes the cost-to-go function for all times t<t_K
     *
	 * \param k step k
	 */
	void computeCostToGo(size_t k);

	//! Design controller
	/*!
	 * This function designes the LQR and feedforward controller at time k.
     *
	 * \param k step k
	 */
	void designController(size_t k);

	//! Compute cost for a given set of state and input trajectory
	/*!
	 * Compute cost for a given set of state and input trajectory
     *
	 * \param threadId the ID of the thread
	 * \param x_local the state trajectory
	 * \param u_local the control trajectory
	 * \param intermediateCost the accumulated intermediate cost
	 * \param finalCost the accumulated final cost
	 */
	void computeCostsOfTrajectory(
			size_t threadId,
			const core::StateVectorArray<STATE_DIM, SCALAR>& x_local,
			const core::ControlVectorArray<CONTROL_DIM, SCALAR>& u_local,
			scalar_t& intermediateCost,
			scalar_t& finalCost
	) const;


	//! Check if controller with particular alpha is better
	void lineSearchSingleController(
			size_t threadId,
			scalar_t alpha,
			ControlVectorArray& u_ff_local,
			ct::core::StateVectorArray<STATE_DIM, SCALAR>& x_local,
			ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>& u_local,
			ct::core::tpl::TimeArray<SCALAR>& t_local,
			scalar_t& intermediateCost,
			scalar_t& finalCost,
			std::atomic_bool* terminationFlag = nullptr
	) const;

	//! Update feedforward controller
	/*!
	 * This function updates the feedforward Controller based on the previous calculation.
     *
	 * \param k step k
	 */
	void updateFFController(size_t k);


	//! Send a std::vector of Eigen to Matlab
	/*!
	 * This is a helper function to efficiently send std::vectors to Matlab.
	 */
	template<class V>
	void matrixToMatlab(V& matrix, std::string variableName);


	//! compute the norm of the difference between two control trajectories
	void computeControlUpdateNorm(const ControlVectorArray& u_prev, const ControlVectorArray& u_new);

	//! compute the norm of the difference between two state trajectories
	void computeStateUpdateNorm(const StateVectorArray& x_prev, const StateVectorArray& x_new);

	//! compute the norm of the defects trajectory
	void computeDefectsNorm();

	typedef std::shared_ptr<ct::core::IntegratorRK4<STATE_DIM, SCALAR> > IntegratorRK4Ptr;
    std::vector<IntegratorRK4Ptr, Eigen::aligned_allocator<IntegratorRK4Ptr> > integratorsRK4_; //! Runge-Kutta-4 Integrators

    typedef std::shared_ptr<ct::core::IntegratorEuler<STATE_DIM, SCALAR> > IntegratorEulerPtr;
    std::vector<IntegratorEulerPtr, Eigen::aligned_allocator<IntegratorEulerPtr> > integratorsEuler_;

	typedef std::shared_ptr<ct::core::IntegratorSymplecticEuler<P_DIM, V_DIM, CONTROL_DIM, SCALAR> > IntegratorSymplecticEulerPtr;
	std::vector<IntegratorSymplecticEulerPtr, Eigen::aligned_allocator<IntegratorSymplecticEulerPtr> > integratorsEulerSymplectic_;

	typedef std::shared_ptr<ct::core::IntegratorSymplecticRk<P_DIM, V_DIM, CONTROL_DIM, SCALAR> > IntegratorSymplecticRkPtr;
	std::vector<IntegratorSymplecticRkPtr, Eigen::aligned_allocator<IntegratorSymplecticRkPtr > > integratorsRkSymplectic_;

    typedef std::shared_ptr<core::ConstantStateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR> > ConstantStateFeedbackControllerPtr;
    std::vector<ConstantStateFeedbackControllerPtr, Eigen::aligned_allocator<ConstantStateFeedbackControllerPtr> > controller_;	//! the constant controller for forward-integration during one time-step



    //! The policy. currently only for returning the result, should eventually replace L_ and u_ff_ (todo)
    NLOCBackendBase::Policy_t policy_;

    ct::core::tpl::TimeArray<SCALAR> t_; //! the time trajectory

    bool initialized_;
    bool configured_;

	size_t iteration_;	/*!< current iteration */

	Settings_t settings_;

	int K_;

	StateVectorArray x_;
	StateVectorArray xShot_;
	StateVectorArray x_prev_;
	ControlVectorArray u_ff_;
	ControlVectorArray u_ff_prev_;
	FeedbackArray L_;

	SCALAR d_norm_; 	// sum of the norms of all defects
	SCALAR lx_norm_; 	// sum of the norms of state update
	SCALAR lu_norm_; 	// sum of the norms of control update


	std::shared_ptr<LQOCProblem<STATE_DIM, CONTROL_DIM, SCALAR> > lqocProblem_;

	std::shared_ptr<LQOCSolver<STATE_DIM, CONTROL_DIM, SCALAR> > lqocSolver_;


	scalar_t intermediateCostBest_;
	scalar_t finalCostBest_;
	scalar_t lowestCost_;

	scalar_t intermediateCostPrevious_;
	scalar_t finalCostPrevious_;


	//! if building with MATLAB support, include matfile
#ifdef MATLAB
	matlab::MatFile matFile_;
#endif //MATLAB


	std::vector<typename OptConProblem_t::DynamicsPtr_t> systems_;
	std::vector<LinearSystemDiscretizer_t> linearSystemDiscretizers_;
	std::vector<typename OptConProblem_t::LinearPtr_t> linearSystems_;
	std::vector<typename OptConProblem_t::CostFunctionPtr_t> costFunctions_;
	std::vector<typename OptConProblem_t::ConstraintPtr_t> stateInputConstraints_;
	std::vector<typename OptConProblem_t::ConstraintPtr_t> pureStateConstraints_;

};


} // namespace optcon
} // namespace ct

#include "implementation/NLOCBackendBase-impl.hpp"


#endif /* INCLUDE_CT_OPTCON_GNMS_GNMSBASE_HPP_ */
