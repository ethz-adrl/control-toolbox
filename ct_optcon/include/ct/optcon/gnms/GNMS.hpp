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

#ifndef INCLUDE_CT_OPTCON_SOLVER_GNMS_H_
#define INCLUDE_CT_OPTCON_SOLVER_GNMS_H_

#include <ct/optcon/solver/NLOptConSettings.hpp>
#include <ct/optcon/nloc/NLOCAlgorithm.hpp>

namespace ct{
namespace optcon{


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR = double>
class GNMS : public NLOCAlgorithm<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>
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
	GNMS(std::shared_ptr<Backend_t>& backend_, const Settings_t& settings) :
		BASE(backend_)
	{}

	//! destructor
	virtual ~GNMS(){}

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
	 * @return foundBetter (false if converged)
	 */
	virtual bool runIteration() override
	{
		prepareIteration();

		return finishIteration();
	}


	/*!
	 * - linearize dynamics for the stages 1 to N-1
	 * - quadratize cost for stages 1 to N-1
	 */
	virtual void prepareIteration() override
	{
		auto startPrepare = std::chrono::steady_clock::now();

		if (!this->backend_->isInitialized())
			throw std::runtime_error("GNMS is not initialized!");

		if (!this->backend_->isConfigured())
			throw std::runtime_error("GNMS is not configured!");

		this->backend_->checkProblem();

		int K = this->backend_->getNumSteps();


		// if first iteration, compute shots and rollout and cost!
		if(this->backend_->iteration() == 0)
		{
//			std::cout << "Running additional init routine for first iteration !!" << std::endl;
			this->backend_->rolloutShots(1, K-1);
		}


		auto start = std::chrono::steady_clock::now();
		this->backend_->computeLinearizedDynamicsAroundTrajectory(1, K-1);
		auto end = std::chrono::steady_clock::now();
		auto diff = end - start;
#ifdef DEBUG_PRINT
		std::cout << "Linearizing from index 1 to N-1 took "<<std::chrono::duration <double, std::milli> (diff).count() << " ms" << std::endl;
#endif


		start = std::chrono::steady_clock::now();
		this->backend_->computeQuadraticCostsAroundTrajectory(1, K-1);
		end = std::chrono::steady_clock::now();
		diff = end - start;
#ifdef DEBUG_PRINT
		std::cout << "Cost computation for index 1 to N-1 took "<<std::chrono::duration <double, std::milli> (diff).count() << " ms" << std::endl;
#endif

		auto endPrepare = std::chrono::steady_clock::now();
#ifdef DEBUG_PRINT
		std::cout << "GNMS prepareIteration() took "<<std::chrono::duration <double, std::milli> (endPrepare-startPrepare).count() << " ms" << std::endl;
#endif


#ifdef DEBUG_PRINT
		std::cout<<"[GNMS]: Solving prepare stage of LQOC Problem"<<std::endl;
#endif // DEBUG_PRINT
		start = std::chrono::steady_clock::now();
		this->backend_->prepareSolveLQProblem();
		end = std::chrono::steady_clock::now();
		diff = end - start;
#ifdef DEBUG_PRINT
		std::cout << "Prepare phase of LQOC problem took "<<std::chrono::duration <double, std::milli> (diff).count() << " ms" << std::endl;
#endif


	} //! prepareIteration()



	//! finish iteration for unconstrained GNMS
	/*!
	 * - linearize dynamcs for the first stage
	 * - quadratize cost for the first stage
	 * @return
	 */
	virtual bool finishIteration() override
	{

		auto startFinish = std::chrono::steady_clock::now();

		int K = this->backend_->getNumSteps();

		// if first iteration, compute shots and rollout and cost!
		if(this->backend_->iteration() == 0)
		{
			this->backend_->rolloutShots(0, 0);
			this->backend_->updateCosts();
		}

#ifdef MATLAB_FULL_LOG
		if (this->backend_->iteration() == 0)
			this->backend_->logInitToMatlab();
#endif

		auto start = std::chrono::steady_clock::now();
		this->backend_->computeLinearizedDynamicsAroundTrajectory(0, 0);
		auto end = std::chrono::steady_clock::now();
		auto diff = end - start;
#ifdef DEBUG_PRINT
		std::cout << "Linearizing for index 0 took "<<std::chrono::duration <double, std::milli> (diff).count() << " ms" << std::endl;
#endif


		start = std::chrono::steady_clock::now();
		this->backend_->computeQuadraticCostsAroundTrajectory(0, 0);
		end = std::chrono::steady_clock::now();
		diff = end - start;
#ifdef DEBUG_PRINT
		std::cout << "Cost computation for index 0 took "<<std::chrono::duration <double, std::milli> (diff).count() << " ms" << std::endl;
#endif


#ifdef DEBUG_PRINT
		std::cout<<"[GNMS]: Finish phase LQOC Problem"<<std::endl;
#endif // DEBUG_PRINT
		start = std::chrono::steady_clock::now();
		this->backend_->finishSolveLQProblem();
		end = std::chrono::steady_clock::now();
		diff = end - start;
#ifdef DEBUG_PRINT
		std::cout << "Finish solving LQOC problem took "<<std::chrono::duration <double, std::milli> (diff).count() << " ms" << std::endl;
#endif

		// update solutions
		this->backend_->updateSolutionState();
		this->backend_->updateSolutionFeedforward();
		this->backend_->updateSolutionFeedback();

		start = std::chrono::steady_clock::now();
		this->backend_->rolloutShots(0, K-1);
		end = std::chrono::steady_clock::now();
		diff = end - start;
#ifdef DEBUG_PRINT
		std::cout << "Shot integration took "<<std::chrono::duration <double, std::milli> (diff).count() << " ms" << std::endl;
#endif

		start = std::chrono::steady_clock::now();
		end = std::chrono::steady_clock::now();
		diff = end - start;
#ifdef DEBUG_PRINT
		std::cout << "Defects computation took "<<std::chrono::duration <double, std::milli> (diff).count() << " ms" << std::endl;
#endif

		// compute new costs
		this->backend_->updateCosts();

		auto endFinish = std::chrono::steady_clock::now();
#ifdef DEBUG_PRINT
		std::cout << "GNMS finishIteration() took "<<std::chrono::duration <double, std::milli> (endFinish-startFinish).count() << " ms" << std::endl;
#endif


#ifdef DEBUG_PRINT
		this->backend_->debugPrint();
#endif //DEBUG_PRINT

#ifdef MATLAB_FULL_LOG
		this->backend_->logToMatlab(this->backend_->iteration());
#endif //MATLAB_FULL_LOG

		this->backend_->iteration()++;

		return (!this->backend_->isConverged());

	} //! finishIteration()


};

}	// namespace optcon
}	// namespace ct

#endif /* INCLUDE_CT_OPTCON_SOLVER_GNMS_H_ */
