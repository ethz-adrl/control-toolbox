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


namespace ct{
namespace optcon{


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendST<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::computeLinearizedDynamicsAroundTrajectory(size_t firstIndex, size_t lastIndex)
{
	for (size_t k=firstIndex; k <= lastIndex; k++)
	{
		this->computeLinearizedDynamics(this->settings_.nThreads, k);
	}
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendST<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::computeQuadraticCostsAroundTrajectory(size_t firstIndex, size_t lastIndex)
{
	this->initializeCostToGo();

	for (size_t k=firstIndex; k<=lastIndex; k++)
	{
		// compute quadratic cost
		this->computeQuadraticCosts(this->settings_.nThreads, k);
	}
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendST<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::rolloutShots(size_t firstIndex, size_t lastIndex)
{
	for (size_t k=firstIndex; k<=lastIndex; k++)
	{
		// first rollout the shot
		this->rolloutSingleShot(this->settings_.nThreads, k);

		// then compute the corresponding defect
		this->computeSingleDefect(this->settings_.nThreads, k);
	}

	this->d_norm_ = this->computeDefectsNorm(this->lqocProblem_->b_);
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
SCALAR NLOCBackendST<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::performLineSearch()
{
	// we start with extrapolation
	double alpha = this->settings_.lineSearchSettings.alpha_0;
	double alphaBest = 0.0;
	size_t iterations = 0;

	while (iterations < this->settings_.lineSearchSettings.maxIterations)
	{
#ifdef DEBUG_PRINT_LINESEARCH
		std::cout<<"[LineSearch]: Iteration: "<< iterations << ", try alpha: "<<alpha<< " out of maximum " << this->settings_.lineSearchSettings.maxIterations << " iterations. "<< std::endl;
#endif

		iterations++;

		SCALAR cost = std::numeric_limits<SCALAR>::max();
		SCALAR intermediateCost = std::numeric_limits<SCALAR>::max();
		SCALAR finalCost = std::numeric_limits<SCALAR>::max();
		SCALAR defectNorm = std::numeric_limits<SCALAR>::max();

		ct::core::StateVectorArray<STATE_DIM, SCALAR> x_search(this->K_+1);
		ct::core::StateVectorArray<STATE_DIM, SCALAR> x_shot_search(this->K_+1);
		ct::core::StateVectorArray<STATE_DIM, SCALAR> defects_recorded(this->K_+1, ct::core::StateVector<STATE_DIM, SCALAR>::Zero());
		ct::core::ControlVectorArray<CONTROL_DIM, SCALAR> u_recorded(this->K_);
		ct::core::tpl::TimeArray<SCALAR> t_search(this->K_+1); // todo get rid of t_search

		//! set init state
		x_search[0] = this->x_[0];



		switch(this->settings_.nlocp_algorithm)
		{
		case NLOptConSettings::NLOCP_ALGORITHM::GNMS :
		{
			this->executeLineSearchMultipleShooting(this->settings_.nThreads, alpha, this->lu_, this->lx_, x_search, x_shot_search, defects_recorded, u_recorded, intermediateCost, finalCost, defectNorm);
			break;
		}
		case NLOptConSettings::NLOCP_ALGORITHM::ILQR :
		{
			defectNorm = 0.0;

			if(this->settings_.closedLoopShooting)
			{
				//! search with lv_ update if we are doing closed-loop shooting
				this->executeLineSearchSingleShooting(this->settings_.nThreads, alpha, this->lv_, x_search, u_recorded, t_search, intermediateCost, finalCost);
			}
			else{
				//! search with whole lu_ update if we are doing closed-loop shooting
				this->executeLineSearchSingleShooting(this->settings_.nThreads, alpha, this->lu_, x_search, u_recorded, t_search, intermediateCost, finalCost);
			}
			break;
		}
		default :
			throw std::runtime_error("Algorithm type unknown in performLineSearch()!");
		}


		cost = intermediateCost + finalCost + this->settings_.meritFunctionRho * defectNorm;

		//! catch the case that a rollout might be unstable
		if(std::isnan(cost) || cost >= this->lowestCost_ ) // todo: alternatively cost >= (this->lowestCost_*(1 - 1e-3*alpha)), study this
		{
#ifdef DEBUG_PRINT_LINESEARCH
			std::cout<<"[LineSearch]: No better cost/merit found at alpha "<< alpha << ":" << std::endl;
			std::cout<<"[LineSearch]: Cost:\t"<<intermediateCost + finalCost<<std::endl;
			std::cout<<"[LineSearch]: Defect:\t"<<defectNorm<<std::endl;
			std::cout<<"[LineSearch]: Merit:\t"<<cost<<std::endl;
#endif //DEBUG_PRINT_LINESEARCH

			//! compute new alpha
			alpha = alpha * this->settings_.lineSearchSettings.n_alpha;
		}
		else
		{
			//! cost < this->lowestCost_ , better merit/cost found!

#ifdef DEBUG_PRINT_LINESEARCH
			std::cout<<"Lower cost/merit found at alpha: "<< alpha << ":" << std::endl;
			std::cout<<"merit: " << cost << "cost "<<intermediateCost + finalCost<<", defect " << defectNorm << " at alpha: "<< alpha << std::endl;
#endif //DEBUG_PRINT_LINESEARCH


#if defined (MATLAB_FULL_LOG) || defined (DEBUG_PRINT)
			this->computeControlUpdateNorm(u_recorded, this->u_ff_prev_);
			this->computeStateUpdateNorm(x_search, this->x_prev_);
#endif

			alphaBest = alpha;
			this->intermediateCostBest_ = intermediateCost;
			this->finalCostBest_ = finalCost;
			this->d_norm_ = defectNorm;
			this->x_prev_ = x_search;
			this->lowestCost_ = cost;
			this->x_.swap(x_search);
			this->xShot_.swap(x_shot_search);
			this->u_ff_.swap(u_recorded);
			this->lqocProblem_->b_.swap(defects_recorded);
			break;
		}
	} //! end while

	return alphaBest;
}


} // namespace optcon
} // namespace ct

