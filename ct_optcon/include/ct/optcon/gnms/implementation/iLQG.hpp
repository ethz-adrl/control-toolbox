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

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void iLQG<STATE_DIM, CONTROL_DIM, SCALAR>::createLQProblem()
{
	this->sequentialLQProblem();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void iLQG<STATE_DIM, CONTROL_DIM, SCALAR>::backwardPass()
{
	// step 3
	// initialize cost to go (described in step 3)
	this->initializeCostToGo();

	for (int k=this->K_-1; k>=0; k--) {
		// design controller
		this->designController(k);

		// compute cost to go
		this->computeCostToGo(k);
	}
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void iLQG<STATE_DIM, CONTROL_DIM, SCALAR>::computeLinearizedDynamicsAroundTrajectory()
{
	for (size_t k=0; k<this->K_; k++)
	{
		// step 2
		// computes linearized dynamics
		this->computeLinearizedDynamics(this->settings_.nThreads, k);
	}
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void iLQG<STATE_DIM, CONTROL_DIM, SCALAR>::computeQuadraticCostsAroundTrajectory()
{
	for (size_t k=0; k<this->K_; k++)
	{
		// compute quadratic cost
		this->computeQuadraticCosts(this->settings_.nThreads, k);
	}
}



template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
SCALAR iLQG<STATE_DIM, CONTROL_DIM, SCALAR>::performLineSearch()
{
#ifdef DEBUG_PRINT_LINESEARCH
	std::cout<<"Starting line search."<<std::endl;
	std::cout<<"Cost last rollout: "<<this->lowestCost_<<std::endl;
#endif //DEBUG_PRINT_LINESEARCH

	// we start with extrapolation
	double alpha = this->settings_.lineSearchSettings.alpha_0;
	double alphaBest = 0.0;
	size_t iterations = 0;

	while (iterations < this->settings_.lineSearchSettings.maxIterations)
	{
#ifdef DEBUG_PRINT_LINESEARCH
		std::cout<<"Iteration: "<< iterations << " with alpha: "<<alpha<< " out of maximum " << this->settings_.lineSearchSettings.maxIterations << " iterations. "<< std::endl;
#endif

		iterations++;

		ct::core::ControlVectorArray<CONTROL_DIM, SCALAR> u_ff_search(this->K_);

		for (int k=this->K_-1; k>=0; k--)
		{
			u_ff_search[k] = alpha * this->lv_[k] + this->u_ff_prev_[k];
		}


		ct::core::StateVectorArray<STATE_DIM, SCALAR> x_search(this->K_+1);
		ct::core::ControlVectorArray<CONTROL_DIM, SCALAR> u_search(this->K_);
		ct::core::tpl::TimeArray<SCALAR> t_search(this->K_+1);
		x_search[0] = this->x_[0];

		bool dynamicsGood = this->rolloutSystem(this->settings_.nThreads, u_ff_search, x_search, u_search, t_search);

		typename Base::scalar_t cost = std::numeric_limits<typename Base::scalar_t>::max();
		typename Base::scalar_t intermediateCost = std::numeric_limits<typename Base::scalar_t>::max();
		typename Base::scalar_t finalCost = std::numeric_limits<typename Base::scalar_t>::max();

		if (dynamicsGood)
		{
			this->computeCostsOfTrajectory(this->settings_.nThreads, x_search, u_search, intermediateCost, finalCost);

			cost = intermediateCost + finalCost;
		}

		if (cost < this->lowestCost_)
		{
			if(std::isnan(cost))
				throw(std::runtime_error("cost is NaN - must not happen since dynamicsGood == true "));

#ifdef DEBUG_PRINT_LINESEARCH
			std::cout<<"Lower cost found: "<< cost <<" at alpha: "<< alpha << std::endl;
#endif //DEBUG_PRINT_LINESEARCH

			this->intermediateCostBest_ = intermediateCost;
			this->finalCostBest_ = finalCost;

			alphaBest = alpha;
			this->lowestCost_ = cost;
			this->x_.swap(x_search);
			this->u_ff_.swap(u_ff_search);
			this->u_.swap(u_search);
			this->t_.swap(t_search);
			break;
		}
		else
		{
#ifdef DEBUG_PRINT_LINESEARCH
			std::cout<<"No better cost found: "<<cost<<" at alpha: "<<alpha<<" so trying again."<<std::endl;
#endif //DEBUG_PRINT_LINESEARCH
		}
		alpha = alpha * this->settings_.lineSearchSettings.n_alpha;
	}

	return alphaBest;
}

