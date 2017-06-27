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

#ifndef ILQGMP_HPP_
#define ILQGMP_HPP_

#include <iostream>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>

#include "iLQGBase.hpp"

namespace ct{
namespace optcon{


//!  Multi-Threaded implementation of iLQG
/*!
 * \ingroup iLQG
 *
 *
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class iLQGMP : public iLQGBase<STATE_DIM, CONTROL_DIM, SCALAR>  {

public:
	// Required as class has Eigen members
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW


	typedef iLQGBase<STATE_DIM, CONTROL_DIM, SCALAR> Base;

	typedef typename Base::Policy_t Policy_t;
	typedef typename Base::Settings_t Settings_t;

	//! iLQG constructor.
	/*!
      Sets up iLQG. Dynamics, derivatives of the dynamics as well as the cost function have to be provided.
      You should pass pointers to instances of classes here that derive from the dynamics, derivatives and costFunction base classes

	 */
	iLQGMP( const OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>& optConProblem,
			const iLQGSettings& settings) :

			Base(optConProblem, settings)
	{
		Eigen::initParallel();
		launchWorkerThreads();
	}

	iLQGMP(const OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>& optConProblem,
		 const std::string& settingsFile,
		 bool verbose = true,
		 const std::string& ns = "ilqg") :
		Base(optConProblem, settingsFile, verbose, ns)
	{
		Eigen::initParallel();
		launchWorkerThreads();
	}

	~iLQGMP();


private:
	void createLQProblem() override;

	void backwardPass() override;

	void computeQuadraticCostsAroundTrajectory() override;

	void computeLinearizedDynamicsAroundTrajectory() override;

	SCALAR performLineSearch() override;


	enum WORKER_STATE {
		IDLE,
		LINE_SEARCH,
		LINEARIZE_DYNAMICS,
		COMPUTE_COST,
		PARALLEL_BACKWARD_PASS,
		SHUTDOWN
	};

	//! Launch all worker thread
	/*!
	  Initializes and launches all worker threads
	 */
	void launchWorkerThreads();

	//! Main function of thread worker
	/*!
	  Includes all tasks that a worker will execute
	 */
	void threadWork(size_t threadId);

	//! Creates the LQ Problem in parallel
	/*!
	  The Dynamics are linearized and the cost function is quadratized in parallel
	 */
	void parallelLQProblem();

	//! Line search for new controller using multi-threading
	/*!
	  Line searches for the best controller in update direction. If line search is disabled, it just takes the suggested update step.
	 */
	void lineSearchWorker(size_t threadId);


	//! Worker function for linearized dynamics
	/*!
	  Gets a parameter k to process and then calls computeLinearizedDynamicsWorker method
	  \param k step k
	 */
	void computeLinearizedDynamicsWorker(size_t threadId);

	//! Computes the linearized dynamics
	/*!
	  This function calculates the linearized dynamics as provided by the derivatives pointer.
	  \param k step k
	 */
	void executeComputeLinearizedDynamics(size_t threadId, size_t k);


	//! Computes the quadratic costs
	/*!
	  This function calculates the quadratic costs as provided by the costFunction pointer.

	  \param k step k
	 */
	void computeQuadraticCostsWorker(size_t threadId);

	//! Creates the linear quadratic problem
	/*!
	  This function calculates the quadratic costs as provided by the costFunction pointer as well as the linearized dynamics.

	  \param k step k
	 */
	void computeLQProblemWorker(size_t threadId);


	std::vector<std::thread, Eigen::aligned_allocator<std::thread>> workerThreads_;
	std::atomic_bool workersActive_;
	std::atomic_int workerTask_;

	std::mutex workerWakeUpMutex_;
	std::condition_variable workerWakeUpCondition_;

	std::mutex kCompletedMutex_;
	std::condition_variable kCompletedCondition_;

	std::mutex kCompletedMutexCost_;
	std::condition_variable kCompletedConditionCost_;

	std::mutex lineSearchResultMutex_;
	std::mutex alphaBestFoundMutex_;
	std::condition_variable alphaBestFoundCondition_;

	std::atomic_size_t alphaTaken_;
	size_t alphaMax_;
	size_t alphaExpBest_;
	size_t alphaExpMax_;
	std::atomic_bool alphaBestFound_;
	std::vector<size_t> alphaProcessed_;

	std::atomic_size_t kTaken_;
	std::atomic_size_t kCompleted_;

	size_t KMax_;

	Eigen::SelfAdjointEigenSolver<typename Base::control_matrix_t> eigenvalueSolver_;

#ifdef MATLAB
	matlab::MatFile matFile_;
#endif //MATLAB
};

} // namespace optcon
} // namespace ct

#include "implementation/iLQGMP.hpp"

#endif /* ILQGMP_HPP_ */
