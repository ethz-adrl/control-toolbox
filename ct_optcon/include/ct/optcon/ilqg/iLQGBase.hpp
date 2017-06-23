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


#ifndef INCLUDE_CT_OPTCON_ILQG_ILQGBASE_HPP_
#define INCLUDE_CT_OPTCON_ILQG_ILQGBASE_HPP_

#include <atomic>

#include <ct/core/core.h>
#include <ct/optcon/costfunction/CostFunctionQuadratic.hpp>
#include <ct/optcon/solver/OptConSolver.h>
#include "iLQGSettings.hpp"

#include "iLQGTester.hpp"

#ifdef MATLAB
#include <ct/optcon/matlab.hpp>
#endif

namespace ct{
namespace optcon{


/*!
 * \defgroup iLQG iLQG
 *
 * \brief Sequential Linear Quadratic Optimal Control package
 */

/*!
 * \ingroup iLQG
 *
 * \brief C++ implementation of iLQG. In fact, this currently implements iLQR.
 *
 *  The implementation and naming is based on the reference below. In general, the code follows this convention:
 *  X  <- Matrix (upper-case in paper)
 *  xv <- vector (lower-case bold in paper)
 *  x  <- scalar (lower-case in paper)
 *
 *  Reference:
 *  Todorov, E.; Weiwei Li, "A generalized iterative LQG method for locally-optimal feedback control of constrained nonlinear stochastic systems,"
 *  American Control Conference, 2005. Proceedings of the 2005 , vol., no., pp.300,306 vol. 1, 8-10 June 2005
*/

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class iLQGBase : public OptConSolver<
		iLQGBase<STATE_DIM, CONTROL_DIM, SCALAR>,
		core::StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>,
		iLQGSettings,
		STATE_DIM,
		CONTROL_DIM,
		SCALAR
	>
{
	//! this is a tester class that tests iLQG and thus needs access to internal members
	friend class iLQGTester<iLQGBase<STATE_DIM, CONTROL_DIM, SCALAR>>;

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	static const size_t STATE_D = STATE_DIM;
	static const size_t CONTROL_D = CONTROL_DIM;

	typedef iLQGSettings Settings_t;
	typedef core::StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR> Policy_t;

	typedef OptConSolver<iLQGBase, Policy_t, iLQGSettings, STATE_DIM, CONTROL_DIM, SCALAR> Base;

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

	typedef core::StateVector<STATE_DIM, SCALAR> state_vector_t;
	typedef core::ControlVector<CONTROL_DIM, SCALAR> control_vector_t;


	typedef SCALAR scalar_t;
	typedef std::vector<SCALAR> scalar_array_t;


    //! iLQG constructor.
    /*!
     * Sets up iLQG. Dynamics, derivatives of the dynamics as well as the cost function have to be provided.
     * You should pass pointers to instances of classes here that derive from the dynamics, derivatives and costFunction base classes
     *
     * \param dynamics pointer to system dynamics
     * \param derivative pointer to system dynamics derivative
     * \param costFunction pointer to cost function and its derivatives
     * \param t_end time horizon for iLQG
     * \param epsilon correction factor for negative eigenvalues in the H matrix
     * \param matlab pointer to Schweizer-Messer Matlab Engine or to a dummy class (if Matlab not used)
    */
	iLQGBase(const OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>& optConProblem,
			const iLQGSettings& settings) :

		    integratorsRK4_(settings.nThreads+1),
		    integratorsEuler_(settings.nThreads+1),

		    controller_(settings.nThreads+1),

			iteration_(0),

			settings_(settings),
			initialized_(false),
			configured_(false),

			K_(0),

			H_corrFix_(ControlMatrix::Zero()),

			lowestCost_(std::numeric_limits<scalar_t>::infinity()),
			intermediateCostBest_(std::numeric_limits<scalar_t>::infinity()),
			finalCostBest_(std::numeric_limits<scalar_t>::infinity()),

			smallestEigenvalue_(std::numeric_limits<scalar_t>::infinity()),
			smallestEigenvalueIteration_(std::numeric_limits<scalar_t>::infinity()),

			eigenvalueSolver_(CONTROL_DIM),

			iLQGTester_(*this)
	{
		for (size_t i=0; i<settings.nThreads+1; i++)
		{
			controller_[i] = std::shared_ptr<core::ConstantController<STATE_DIM, CONTROL_DIM, SCALAR> > (new core::ConstantController<STATE_DIM, CONTROL_DIM, SCALAR>());
		}

		configure(settings);
		this->setProblem(optConProblem);
	}

	iLQGBase(const OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>& optConProblem,
			 const std::string& settingsFile,
			 bool verbose = true,
			 const std::string& ns = "ilqg") :
			iLQGBase(optConProblem, iLQGSettings::fromConfigFile(settingsFile, verbose, ns))
	{
	}


	virtual ~iLQGBase() {};


	//! configure the solver
	/**
	 * Configure the solver
	 * @param settings solver settings
	 */
	virtual void configure(const iLQGSettings& settings) override;


	//! get the current SLQsolver settings
	const iLQGSettings& getSettings() const { return settings_; }


	//! reset iLQG
	/*!
	 * \brief reset iLQG
	 * resets iLQG by setting the iteration count to zero, forcing a new rollout during the next iteration.
	 */
	void reset() {
		iteration_ = 0;
		smallestEigenvalue_ = std::numeric_limits<scalar_t>::infinity();
		smallestEigenvalueIteration_ = std::numeric_limits<scalar_t>::infinity();
	}


	/*!
	 * solve, returns true if solve succeeded, false otherwise.
	 * */
	bool solve() override;

	/*!
	 * run a single iteration of the solver (might not be supported by all solvers)
	 * @return true if a better solution was found
	 */
	bool runIteration() override;

	/*!
	 * Get the solution to the optimal control problem
	 * @return
	 */
	const Policy_t& getSolution() override;


	/*!
	 * Get the optimized trajectory to the optimal control problem
	 * @return
	 */
	const core::StateTrajectory<STATE_DIM, SCALAR> getStateTrajectory() const override { return core::StateTrajectory<STATE_DIM, SCALAR>(t_, x_); }


	/*!
	 * Get the optimized trajectory to the optimal control problem
	 * @return
	 */
	const core::StateVectorArray<STATE_DIM, SCALAR>& getStates() const { return  x_; }


	/*!
	 * Get the optimal control input corresponding to the optimal trajectory.
	 * This needs to return u, since this is the combined feedforward-feedback
	 * control that generates the optimal trajectory.
	 * @return
	 */
	const core::ControlTrajectory<CONTROL_DIM, SCALAR, SCALAR> getControlTrajectory() const override;


	/*!
	 * Get the time indeces corresponding to the solution
	 * @return
	 */
	const core::tpl::TimeArray<SCALAR>& getTimeArray() const { return t_; }


	/*! get the current Optimal Control Problem Time Horizon
	 *
	 * @return time horizon
	 */
	SCALAR getTimeHorizon() const override { return K_*settings_.dt; }


	/*!
	 * Set the initial guess used by the solver (not all solvers might support initial guesses)
	 */
	void setInitialGuess(const Policy_t& initialGuess) override;


	/*!
	 * \brief Change the time horizon the solver operates on.
	 *
	 * This function does not need to be called if setOptConProblem() has been called
	 * with an OptConProblem that had the correct time horizon set.
	 */
	void changeTimeHorizon(const SCALAR& tf) override;


	/*!
	 * \brief Change the initial state for the optimal control problem
	 *
	 * This function does not need to be called if setOptConProblem() has been called
	 * with an OptConProblem that had the correct initial state set
	 */
	void changeInitialState(const core::StateVector<STATE_DIM, SCALAR>& x0) override;

	/*!
	 * \brief Change the cost function
	 *
	 * This function does not need to be called if setOptConProblem() has been called
	 * with an OptConProblem that had the correct cost function
	 */
	void changeCostFunction(const typename Base::OptConProblem_t::CostFunctionPtr_t& cf) override;

	/*!
	 * \brief Change the nonlinear system
	 *
	 * This function does not need to be called if setOptConProblem() has been called
	 * with an OptConProblem that had the correct nonlinear system
	 */
	void changeNonlinearSystem(const typename Base::OptConProblem_t::DynamicsPtr_t& dyn) override;

	/*!
	 * \brief Change the linear system
	 *
	 * This function does not need to be called if setOptConProblem() has been called
	 * with an OptConProblem that had the correct linear system
	 */
	void changeLinearSystem(const typename Base::OptConProblem_t::LinearPtr_t& lin) override;


	/*!
	 * Tests consistency of the instance of the dynamics, linear systems and costs. This is not a test for thread safety.
	 * @return returns true if instances are consistent with each other
	 */
	bool testConsistency()
	{
		return iLQGTester_.testAllConsistency(true);
	}


	//! Retrieve Last Linearized Model
	/*!
	  Retrieve the linearized model computed during the last iteration
	*/
	void retrieveLastLinearizedModel(StateMatrixArray& A, StateControlMatrixArray& B);


	//! Export all functions to matlab workspace
	/*!
	  This function can be used for Debugging. It exports all variables to Matlab after each iteration. It also saves
	  the Matlab workspace to a .mat file.
	*/
	void logToMatlab();

	SCALAR getCost() const override;

protected:
	virtual void createLQProblem() = 0;

	virtual void backwardPass() = 0;

	//! Computes the linearization of the dynamics along the trajectory. See computeLinearizedDynamics for details
	virtual void computeLinearizedDynamicsAroundTrajectory() = 0;

	//! Computes the quadratic approximation of the cost function along the trajectory
	virtual void computeQuadraticCostsAroundTrajectory() = 0;

	virtual SCALAR performLineSearch() = 0;

	//! check problem for consistency
	void checkProblem();

	//! run the forward pass (forward rollout and linear-quadratic approximation)
	bool forwardPass();

	//! build the sequential LQ problems
	void sequentialLQProblem();

	//! perform line-search and update controller
	bool lineSearchController();

	//! learn the currently optimal line-search parameter (for adaptive line-search)
	SCALAR learnAlpha(const SCALAR& alpha);

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

	//! Print debugging information
	/*!
	 *  This function is automatically called if the DEBUG_PRINT compileflag is set. It prints out important information
	 *  like cost etc. after each iteration.
     *
	 */
	void debugPrint();

	//! Send a std::vector of Eigen to Matlab
	/*!
	 * This is a helper function to efficiently send std::vectors to Matlab.
	 */
	template<class V>
	void matrixToMatlab(V& matrix, std::string variableName);


	typedef std::shared_ptr<ct::core::IntegratorRK4<STATE_DIM, SCALAR> > IntegratorRK4Ptr;
    std::vector<IntegratorRK4Ptr, Eigen::aligned_allocator<IntegratorRK4Ptr> > integratorsRK4_; //! Runge-Kutta-4 Integrators

    typedef std::shared_ptr<ct::core::IntegratorEuler<STATE_DIM, SCALAR> > IntegratorEulerPtr;
    std::vector<IntegratorEulerPtr, Eigen::aligned_allocator<IntegratorEulerPtr> > integratorsEuler_;

	typedef std::shared_ptr<ct::core::IntegratorSymplecticEuler<STATE_DIM / 2, STATE_DIM / 2, CONTROL_DIM, SCALAR> > IntegratorSymplecticEulerPtr;
	std::vector<IntegratorSymplecticEulerPtr, Eigen::aligned_allocator<IntegratorSymplecticEulerPtr> > integratorsEulerSymplectic_;

	typedef std::shared_ptr<ct::core::IntegratorSymplecticRk<STATE_DIM / 2, STATE_DIM / 2, CONTROL_DIM, SCALAR> > IntegratorSymplecticRkPtr;
	std::vector<IntegratorSymplecticRkPtr, Eigen::aligned_allocator<IntegratorSymplecticRkPtr > > integratorsRkSymplectic_;

    typedef std::shared_ptr<core::ConstantController<STATE_DIM, CONTROL_DIM, SCALAR> > ConstantControllerPtr;
    std::vector<ConstantControllerPtr, Eigen::aligned_allocator<ConstantControllerPtr> > controller_;	//! the constant controller for forward-integration during one time-step

    //! The policy. currently only for returning the result, should eventually replace L_ and u_ff_ (todo)
    iLQGBase::Policy_t policy_;

    ct::core::tpl::TimeArray<SCALAR> t_; //! the time trajectory

    bool initialized_;
    bool configured_;

	size_t iteration_;	/*!< current iteration */

	iLQGSettings settings_;

	size_t K_;

	StateMatrixArray A_;
	StateControlMatrixArray B_;

	StateVectorArray x_;
	ControlVectorArray u_;
	ControlVectorArray u_ff_;
	ControlVectorArray u_ff_prev_;

	ControlVectorArray gv_;
	FeedbackArray G_;

	ControlMatrixArray H_;
	ControlMatrixArray Hi_;
	ControlMatrixArray Hi_inverse_;
	ControlMatrix H_corrFix_;

	ControlVectorArray lv_;
	FeedbackArray L_;

	FeedbackArray P_;

	scalar_array_t q_;
	StateVectorArray qv_;
	StateMatrixArray Q_;

	ControlVectorArray rv_;
	ControlMatrixArray R_;

	StateVectorArray sv_;
	StateMatrixArray S_;

	scalar_t intermediateCostBest_;
	scalar_t finalCostBest_;
	scalar_t lowestCost_;

	scalar_t smallestEigenvalue_;
	scalar_t smallestEigenvalueIteration_;

	Eigen::SelfAdjointEigenSolver<control_matrix_t> eigenvalueSolver_; //! Eigenvalue solver, used for inverting the Hessian and for regularization

	iLQGTester<iLQGBase<STATE_DIM, CONTROL_DIM, SCALAR>> iLQGTester_;

#ifdef MATLAB
	matlab::MatFile matFile_;
#endif //MATLAB
};


} // namespace optcon
} // namespace ct

#include "implementation/iLQGBase.hpp"


#endif /* INCLUDE_CT_OPTCON_ILQG_ILQGBASE_HPP_ */
