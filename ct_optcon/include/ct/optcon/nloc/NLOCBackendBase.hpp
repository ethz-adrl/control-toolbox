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


#ifndef INCLUDE_CT_OPTCON_GNMS_GNMSBASE_HPP_
#define INCLUDE_CT_OPTCON_GNMS_GNMSBASE_HPP_

#include <atomic>

#include <ct/core/core.h>
#include <ct/optcon/costfunction/CostFunctionQuadratic.hpp>
#include <ct/optcon/solver/OptConSolver.h>

#include "GNMSSettings.hpp"

#ifdef MATLAB
#include <ct/optcon/matlab.hpp>
#endif

namespace ct{
namespace optcon{


/*!
 * \defgroup GNMS GNMS
 *
 * \brief Sequential Linear Quadratic Optimal Control package
 */

/*!
 * \ingroup GNMS
 *
 * \brief C++ implementation of GNMS.
 *
 *  The implementation and naming is based on the reference below. In general, the code follows this convention:
 *  X  <- Matrix (upper-case in paper)
 *  xv <- vector (lower-case bold in paper)
 *  x  <- scalar (lower-case in paper)
 *
 *  Reference:
*/

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM = STATE_DIM/2, size_t V_DIM=STATE_DIM/2, typename SCALAR = double>
class NLOCBackendBase
{
	static_assert(P_DIM + V_DIM == STATE_DIM, "symplectic dimensions should add up to state dimension");

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	static const size_t state_dim = STATE_DIM;
	static const size_t control_dim = CONTROL_DIM;

	typedef GNMSSettings Settings_t;
	typedef core::ConstantTrajectoryController<STATE_DIM, CONTROL_DIM, SCALAR> Policy_t;

	typedef OptConSolver<NLOCBackendBase, Policy_t, GNMSSettings, STATE_DIM, CONTROL_DIM, SCALAR> Base;

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
	typedef std::vector<SCALAR, Eigen::aligned_allocator<SCALAR>> scalar_array_t;



	NLOCBackendBase(const OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>& optConProblem,
			const GNMSSettings& settings) :

		    integratorsRK4_(settings.nThreads+1),
		    integratorsEuler_(settings.nThreads+1),
			integratorsEulerSymplectic_(settings.nThreads+1),
			integratorsRkSymplectic_(settings.nThreads+1),

		    controller_(settings.nThreads+1)

	{
		for (size_t i=0; i<settings.nThreads+1; i++)
		{
			controller_[i] = std::shared_ptr<core::ConstantController<STATE_DIM, CONTROL_DIM, SCALAR> > (new core::ConstantController<STATE_DIM, CONTROL_DIM, SCALAR>());
		}

		configure(settings);
		this->setProblem(optConProblem);
	}

	NLOCBackendBase(const OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>& optConProblem,
			 const std::string& settingsFile,
			 bool verbose = true,
			 const std::string& ns = "ilqg") :
			NLOCBackendBase(optConProblem, GNMSSettings::fromConfigFile(settingsFile, verbose, ns))
	{
	}


	virtual ~NLOCBackendBase() {};


	//! configure the solver
	/**
	 * Configure the solver
	 * @param settings solver settings
	 */
	virtual void configure(const GNMSSettings& settings) override;


	//! get the current SLQsolver settings
	const GNMSSettings& getSettings() const { return settings_; }


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
		return true;
	}


	//! Export all functions to matlab workspace
	/*!
	  This function can be used for Debugging. It exports all variables to Matlab after each iteration. It also saves
	  the Matlab workspace to a .mat file.
	*/
	void logToMatlab(const size_t& iteration);

	void logInitToMatlab();

	SCALAR getCost() const override;

	SCALAR getTotalDefect() const { return d_norm_;}

protected:

	virtual void createLQProblem() = 0;

	virtual void backwardPass() = 0;

	#ifdef HPIPM
	void backwardPassHPIPM()
	{
		HPIPMInterface_.solve();
	}
	#endif

	//! Computes the linearization of the dynamics along the trajectory. See computeLinearizedDynamics for details
	virtual void computeLinearizedDynamicsAroundTrajectory() = 0;

	//! Computes the quadratic approximation of the cost function along the trajectory
	virtual void computeQuadraticCostsAroundTrajectory() = 0;

	virtual void updateControlAndState() = 0;

	virtual void updateShots() = 0;

	virtual void initializeShots() = 0;

	virtual void computeDefects() = 0;

	//! check problem for consistency
	void checkProblem();

	//! build the sequential LQ problems
	void sequentialLQProblem();

	//! updates the controls and states
	void updateSingleControlAndState(size_t threadId, size_t k);

	//! integrate the individual shots
	void initializeSingleShot(size_t threadId, size_t k);

	//! integrate the individual shots
	void updateSingleShot(size_t threadId, size_t k);

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


	//! Design the state update
	void designStateUpdate(size_t k);


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

	typedef std::shared_ptr<ct::core::IntegratorSymplecticEuler<P_DIM, V_DIM, CONTROL_DIM, SCALAR> > IntegratorSymplecticEulerPtr;
	std::vector<IntegratorSymplecticEulerPtr, Eigen::aligned_allocator<IntegratorSymplecticEulerPtr> > integratorsEulerSymplectic_;

	typedef std::shared_ptr<ct::core::IntegratorSymplecticRk<P_DIM, V_DIM, CONTROL_DIM, SCALAR> > IntegratorSymplecticRkPtr;
	std::vector<IntegratorSymplecticRkPtr, Eigen::aligned_allocator<IntegratorSymplecticRkPtr > > integratorsRkSymplectic_;

    typedef std::shared_ptr<core::ConstantController<P_DIM, V_DIM, SCALAR> > ConstantControllerPtr;
    std::vector<ConstantControllerPtr, Eigen::aligned_allocator<ConstantControllerPtr> > controller_;	//! the constant controller for forward-integration during one time-step

    //! The policy. currently only for returning the result, should eventually replace L_ and u_ff_ (todo)
    NLOCBackendBase::Policy_t policy_;

    ct::core::tpl::TimeArray<SCALAR> t_; //! the time trajectory

    bool initialized_;
    bool configured_;

	size_t iteration_;	/*!< current iteration */

	GNMSSettings settings_;

	int K_;

	StateVectorArray x_;
	StateVectorArray xShot_;
	ControlVectorArray u_ff_;
	ControlVectorArray u_ff_prev_;
	SCALAR dx_norm_;
	SCALAR du_norm_;

	StateVectorArray d_; // the defects
	SCALAR d_norm_; 	// sum of the norms of all defects



	std::shared_ptr<LQOCProblem<STATE_DIM, CONTROL_DIM, SCALAR> > lqocProblem_;


	scalar_t intermediateCostBest_;
	scalar_t finalCostBest_;
	scalar_t lowestCost_;

	scalar_t intermediateCostPrevious_;
	scalar_t finalCostPrevious_;

	scalar_t smallestEigenvalue_;
	scalar_t smallestEigenvalueIteration_;

#ifdef HPIPM
	HPIPMInterface<STATE_DIM, CONTROL_DIM> HPIPMInterface_;
#endif



#ifdef MATLAB
	matlab::MatFile matFile_;
#endif //MATLAB
};


} // namespace optcon
} // namespace ct

#include "implementation/NLOCBackendBase-impl.hpp"


#endif /* INCLUDE_CT_OPTCON_GNMS_GNMSBASE_HPP_ */
