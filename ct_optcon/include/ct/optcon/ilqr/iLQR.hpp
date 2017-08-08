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

#ifndef INCLUDE_CT_OPTCON_SOLVER_ILQR_H_
#define INCLUDE_CT_OPTCON_SOLVER_ILQR_H_

#include <ct/optcon/solver/NLOptConSettings.hpp>
#include <ct/optcon/nloc/NLOCAlgorithm.hpp>

namespace ct{
namespace optcon{


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR = double>
class iLQR : public NLOCAlgorithm<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	static const size_t STATE_D = STATE_DIM;
	static const size_t CONTROL_D = CONTROL_DIM;

	typedef ct::core::StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR> Policy_t;
	typedef NLOptConSettings Settings_t;
	typedef SCALAR Scalar_t;

	typedef NLOCAlgorithm<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR> BASE;
	typedef NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR> Backend_t;

	//! constructor
	iLQR(std::shared_ptr<Backend_t>& backend_, const Settings_t& settings) :
		BASE(backend_)
	{}

	//! destructor
	virtual ~iLQR(){}

	//! configure the solver
	virtual void configure(const Settings_t& settings) override
	{
		this->backend_->configure(settings);
	}

	//! set an initial guess
	virtual void setInitialGuess(const Policy_t& initialGuess) override
	{
		this->backend_->setInitialGuess(initialGuess);
	}


	//! runIteration combines prepareIteration and finishIteration
	/*!
	 * For iLQR the separation between prepareIteration and finishIteration would actually not be necessary
	 * @return
	 */
	virtual bool runIteration() override
	{
		prepareIteration();

		return finishIteration();
	}


	/*!
	 * for iLQR, as it is a purely sequential approach, we cannot prepare anything prior to solving,
	 */
	virtual void prepareIteration() override {

		if (!this->backend_->isInitialized())
			throw std::runtime_error("iLQR is not initialized!");

		if (!this->backend_->isConfigured())
			throw std::runtime_error("iLQR is not configured!");

		this->backend_->checkProblem();
	}


	/*!
	 * for iLQR, finishIteration contains the whole main iLQR iteration.
	 * @return
	 */
	virtual bool finishIteration() override
	{

		int K = this->backend_->getNumSteps();

		// if first iteration, compute shots and rollout and cost!
		if(this->backend_->iteration() == 0)
		{
			if(!this->backend_->nominalRollout())
				throw std::runtime_error("Rollout failed. System became unstable");

			this->backend_->updateCosts();
		}

#ifdef MATLAB_FULL_LOG
		if (this->backend_->iteration() == 0)
			this->backend_->logInitToMatlab();
#endif

		auto start = std::chrono::steady_clock::now();
		auto startEntire = start;
		//! linearize dynamics around the whole trajectory
		this->backend_->computeLinearizedDynamicsAroundTrajectory(0, K-1);
		auto end = std::chrono::steady_clock::now();
		auto diff = end - start;
#ifdef DEBUG_PRINT
		std::cout << "[iLQR]: Linearizing dynamics took "<<std::chrono::duration <double, std::milli> (diff).count() << " ms" << std::endl;
#endif

		start = std::chrono::steady_clock::now();
		//! compute the quadratic cost around the whole trajectory
		this->backend_->computeQuadraticCostsAroundTrajectory(0, K-1);
		end = std::chrono::steady_clock::now();
		diff = end - start;
#ifdef DEBUG_PRINT
		std::cout << "[iLQR]: Cost computation took "<<std::chrono::duration <double, std::milli> (diff).count() << " ms" << std::endl;
#endif


		end = std::chrono::steady_clock::now();
		diff = end - startEntire;
#ifdef DEBUG_PRINT
		std::cout << "[iLQR]: Forward pass took "<<std::chrono::duration <double, std::milli> (diff).count() << " ms" << std::endl;
#endif


#ifdef DEBUG_PRINT
		std::cout<<"[iLQR]: #2 Solve LQOC Problem"<<std::endl;
#endif
		start = std::chrono::steady_clock::now();
		this->backend_->solveFullLQProblem();
		end = std::chrono::steady_clock::now();
		diff = end - start;
#ifdef DEBUG_PRINT
		std::cout << "[iLQR]: Solving LQOC problem took "<<std::chrono::duration <double, std::milli> (diff).count() << " ms" << std::endl;
#endif

		// update solutions
		this->backend_->getFeedforwardUpdates();// todo only when closed loop shooting
		this->backend_->getFeedback(); 			// todo only when closed loop shooting
		this->backend_->getControlUpdates(); 	// todo only when open loop shooting
		this->backend_->getStateUpdates();

		// line-search
#ifdef DEBUG_PRINT
		std::cout<<"[iLQR]: #3 LineSearch"<<std::endl;
#endif

		start = std::chrono::steady_clock::now();
		bool foundBetter = this->backend_->lineSearchSingleShooting();
		end = std::chrono::steady_clock::now();
		diff = end - start;
#ifdef DEBUG_PRINT
		std::cout << "[iLQR]: Line search took "<<std::chrono::duration <double, std::milli> (diff).count() << " ms" << std::endl;
#endif

		diff = end - startEntire;
#ifdef DEBUG_PRINT
		std::cout << "[iLQR]: finishIteration took "<<std::chrono::duration <double, std::milli> (diff).count() << " ms" << std::endl;
#endif

#ifdef DEBUG_PRINT
		this->backend_->debugPrint();
#endif

#ifdef MATLAB_FULL_LOG
		this->backend_->logToMatlab(this->backend_->iteration());
#endif

		this->backend_->iteration()++;

		return foundBetter;
	}


};

}	// namespace optcon
}	// namespace ct

#endif /* INCLUDE_CT_OPTCON_SOLVER_ILQR_H_ */
