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

namespace ct {
namespace optcon {


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
iLQGMP<STATE_DIM, CONTROL_DIM, SCALAR>::~iLQGMP()
{
	workersActive_ = false;
	workerTask_ = SHUTDOWN;
	workerWakeUpCondition_.notify_all();

#ifdef DEBUG_PRINT_MP
	std::cout<<"Shutting down workers"<<std::endl;
#endif // DEBUG_PRINT_MP

	for (size_t i=0; i<workerThreads_.size(); i++)
	{
		workerThreads_[i].join();
	}

#ifdef DEBUG_PRINT_MP
	std::cout<<"All workers shut down"<<std::endl;
#endif // DEBUG_PRINT_MP
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void iLQGMP<STATE_DIM, CONTROL_DIM, SCALAR>::threadWork(size_t threadId)
{
#ifdef DEBUG_PRINT_MP
	std::cout<<"[Thread "<<threadId<<"]: launched"<<std::endl;
#endif // DEBUG_PRINT_MP

	WORKER_STATE lastCompletedTask = IDLE;

	while(workersActive_)
	{
#ifdef DEBUG_PRINT_MP
		std::cout<<"[Thread "<<threadId<<"]: last completed job was: "<<lastCompletedTask<<std::endl;
		std::cout<<"[Thread "<<threadId<<"]: current job is: "<<workerTask_<<std::endl;
#endif // DEBUG_PRINT_MP

		// if job completed in time or IDLE then wait
		if (lastCompletedTask == workerTask_ || workerTask_ == IDLE)
		{
			std::unique_lock<std::mutex> waitLock(workerWakeUpMutex_);
			workerWakeUpCondition_.wait(waitLock, [this, lastCompletedTask]{
				return (workerTask_ != IDLE && lastCompletedTask != workerTask_);
			});
			waitLock.unlock();
		}

#ifdef DEBUG_PRINT_MP
		std::cout<<"[Thread "<<threadId<<"]: woke up!"<<std::endl;
#endif // DEBUG_PRINT_MP

		if (!workersActive_)
			break;

		switch((int)workerTask_)
		{
		case LINE_SEARCH:
		{
#ifdef DEBUG_PRINT_MP
			std::cout<<"[Thread "<<threadId<<"]: now doing line search!"<<std::endl;
#endif // DEBUG_PRINT_MP
			lineSearchWorker(threadId);
			lastCompletedTask = LINE_SEARCH;
			break;
		}
		case LINEARIZE_DYNAMICS:
		{
#ifdef DEBUG_PRINT_MP
			std::cout<<"[Thread "<<threadId<<"]: now doing linearization!"<<std::endl;
#endif // DEBUG_PRINT_MP
			computeLinearizedDynamicsWorker(threadId);
			lastCompletedTask = LINEARIZE_DYNAMICS;
			break;
		}
		case COMPUTE_COST:
		{
#ifdef DEBUG_PRINT_MP
			std::cout<<"[Thread "<<threadId<<"]: now doing cost computation!"<<std::endl;
#endif // DEBUG_PRINT_MP
			computeQuadraticCostsWorker(threadId);
			lastCompletedTask = COMPUTE_COST;
			break;
		}

		case PARALLEL_BACKWARD_PASS:
		{
			if (threadId < this->settings_.nThreads-1)
			{
#ifdef DEBUG_PRINT_MP
			std::cout<<"[Thread "<<threadId<<"]: now doing LQ problem building!"<<std::endl;
#endif // DEBUG_PRINT_MP
				computeLQProblemWorker(threadId);
			}
			lastCompletedTask = PARALLEL_BACKWARD_PASS;
			break;
		}

		case SHUTDOWN:
		{
#ifdef DEBUG_PRINT_MP
			std::cout<<"[Thread "<<threadId<<"]: now shutting down!"<<std::endl;
#endif // DEBUG_PRINT_MP
			return;
			break;
		}

		case IDLE:
		{
#ifdef DEBUG_PRINT_MP
			std::cout<<"[Thread "<<threadId<<"]: is idle, going to sleep!"<<std::endl;
#endif // DEBUG_PRINT_MP
			break;
		}

		default:
		{
			std::cout << "Warning, worker task has unknown task"<<std::endl;
			break;
		}
		}
#ifdef DEBUG_PRINT_MP
		std::cout<<"[Thread "<<threadId<<"]: done with job. Will wait for next now!"<<std::endl;
#endif // DEBUG_PRINT_MP
	}
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void iLQGMP<STATE_DIM, CONTROL_DIM, SCALAR>::launchWorkerThreads()
{
	workersActive_ = true;
	workerTask_ = IDLE;

	for (size_t i=0; i<this->settings_.nThreads; i++)
	{
		workerThreads_.push_back(std::thread(&iLQGMP::threadWork, this, i));
	}
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void iLQGMP<STATE_DIM, CONTROL_DIM, SCALAR>::createLQProblem()
{
	if (this->settings_.parallelBackward.enabled)
		parallelLQProblem();
	else
		this->sequentialLQProblem();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void iLQGMP<STATE_DIM, CONTROL_DIM, SCALAR>::parallelLQProblem()
{
	Eigen::setNbThreads(1); // disable Eigen multi-threading

	kTaken_ = 0;
	kCompleted_ = 0;
	KMax_ = this->K_;

#ifdef DEBUG_PRINT_MP
	std::cout<<"[MP]: Waking up workers to do parallel backward pass. Will continue immediately"<<std::endl;
#endif //DEBUG_PRINT_MP
	workerTask_ = PARALLEL_BACKWARD_PASS;
	workerWakeUpCondition_.notify_all();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void iLQGMP<STATE_DIM, CONTROL_DIM, SCALAR>::backwardPass()
{
	// step 3
	// initialize cost to go (described in step 3)
	this->initializeCostToGo();

	if (this->settings_.parallelBackward.enabled)
	{
		while (kCompleted_ < this->settings_.nThreads*2 && kCompleted_ < this->K_)
		{
			if (this->settings_.parallelBackward.showWarnings)
			{
				std::cout << "backward pass waiting for head start" << std::endl;
			}
			std::this_thread::sleep_for(std::chrono::microseconds(this->settings_.parallelBackward.pollingTimeoutUs));
		}
	}

	for (int k=this->K_-1; k>=0; k--) {

		if (this->settings_.parallelBackward.enabled)
		{
			while ((this->K_-1 - k + this->settings_.nThreads*2 > kCompleted_) && (k >= this->settings_.nThreads*2))
			{
				if (this->settings_.parallelBackward.showWarnings)
				{
					std::cout << "backward pass waiting for LQ problems" << std::endl;
				}
				std::this_thread::sleep_for(std::chrono::microseconds(this->settings_.parallelBackward.pollingTimeoutUs));
			}
		}

#ifdef DEBUG_PRINT_MP
		if (k%100 == 0)
			std::cout<<"[MP]: Solving backward pass for index k "<<k<<std::endl;
#endif

		// design controller
		this->designController(k);

		// compute cost to go
		this->computeCostToGo(k);
	}

	workerTask_ = IDLE;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
SCALAR iLQGMP<STATE_DIM, CONTROL_DIM, SCALAR>::performLineSearch()
{
	Eigen::setNbThreads(1); // disable Eigen multi-threading

		alphaProcessed_.clear();
		alphaTaken_ = 0;
		alphaBestFound_ = false;
		alphaExpBest_ = this->settings_.lineSearchSettings.maxIterations;
		alphaExpMax_ = this->settings_.lineSearchSettings.maxIterations;
		alphaProcessed_.resize(this->settings_.lineSearchSettings.maxIterations, 0);

#ifdef DEBUG_PRINT_MP
		std::cout<<"[MP]: Waking up workers."<<std::endl;
#endif //DEBUG_PRINT_MP
		workerTask_ = LINE_SEARCH;
		workerWakeUpCondition_.notify_all();

#ifdef DEBUG_PRINT_MP
		std::cout<<"[MP]: Will sleep now until we have results."<<std::endl;
#endif //DEBUG_PRINT_MP
		std::unique_lock<std::mutex> waitLock(alphaBestFoundMutex_);
		alphaBestFoundCondition_.wait(waitLock, [this]{return alphaBestFound_.load();});
		waitLock.unlock();
		workerTask_ = IDLE;
#ifdef DEBUG_PRINT_MP
		std::cout<<"[MP]: Woke up again, should have results now."<<std::endl;
#endif //DEBUG_PRINT_MP

		double alphaBest = 0.0;
		if (alphaExpBest_ != alphaExpMax_)
		{
			alphaBest = this->settings_.lineSearchSettings.alpha_0 * std::pow(this->settings_.lineSearchSettings.n_alpha, alphaExpBest_);
		}

		return alphaBest;

} // end linesearch


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void iLQGMP<STATE_DIM, CONTROL_DIM, SCALAR>::lineSearchWorker(size_t threadId)
{
	while(true)
	{
		size_t alphaExp = alphaTaken_++;

#ifdef DEBUG_PRINT_MP
		std::cout<<"[Thread "<<threadId<<"]: Taking alpha index "<<alphaExp<<std::endl;
#endif

		if (alphaExp >= alphaExpMax_ || alphaBestFound_)
		{
			return;
		}

		// convert to real alpha
		double alpha = this->settings_.lineSearchSettings.alpha_0 * std::pow(this->settings_.lineSearchSettings.n_alpha, alphaExp);
		typename Base::StateVectorArray x_local(1);
		typename Base::ControlVectorArray u_local;
		typename Base::TimeArray t_local;

		x_local[0] = this->x_[0];

		SCALAR intermediateCost;
		SCALAR finalCost;

		typename Base::ControlVectorArray u_ff_local(this->K_);
		this->lineSearchSingleController(threadId, alpha, u_ff_local, x_local, u_local, t_local, intermediateCost, finalCost, &alphaBestFound_);

		SCALAR cost = intermediateCost + finalCost;

		lineSearchResultMutex_.lock();
		if (cost < this->lowestCost_)
		{
			// make sure we do not alter an existing result
			if (alphaBestFound_)
			{
				lineSearchResultMutex_.unlock();
				break;
			}

#ifdef DEBUG_PRINT_LINESEARCH
			std::cout<<"[LineSearch, Thread "<<threadId<<"]: Lower cost found: "<<cost<<" at alpha: "<<alpha<<std::endl;
#endif //DEBUG_PRINT_LINESEARCH

			alphaExpBest_ = alphaExp;
			this->intermediateCostBest_ = intermediateCost;
			this->finalCostBest_ = finalCost;
			this->lowestCost_ = cost;
			this->x_.swap(x_local);
			this->u_.swap(u_local);
			this->u_ff_.swap(u_ff_local);
			this->t_.swap(t_local);
		} else
		{
#ifdef DEBUG_PRINT_LINESEARCH
			std::cout<<"[LineSearch, Thread "<<threadId<<"]: No lower cost found, cost "<<cost<<" at alpha "<<alpha<<" . Best cost was "<<this->lowestCost_ <<std::endl;
#endif //DEBUG_PRINT_LINESEARCH
		}

		alphaProcessed_[alphaExp] = 1;

		// we now check if all alphas prior to the best have been processed
		// this also covers the case that there is no better alpha
		bool allPreviousAlphasProcessed = true;
		for (size_t i=0; i<alphaExpBest_; i++)
		{
			if (alphaProcessed_[i] != 1)
			{
				allPreviousAlphasProcessed = false;
				break;
			}
		}
		if (allPreviousAlphasProcessed)
		{
			alphaBestFound_ = true;
			alphaBestFoundCondition_.notify_all();
		}

		lineSearchResultMutex_.unlock();
	}
}



template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void iLQGMP<STATE_DIM, CONTROL_DIM, SCALAR>::computeLinearizedDynamicsAroundTrajectory()
{
	Eigen::setNbThreads(1); // disable Eigen multi-threading

	kTaken_ = 0;
	kCompleted_ = 0;
	KMax_ = this->K_;

#ifdef DEBUG_PRINT_MP
	std::cout<<"[MP]: Waking up workers to do linearization."<<std::endl;
#endif //DEBUG_PRINT_MP
	workerTask_ = LINEARIZE_DYNAMICS;
	workerWakeUpCondition_.notify_all();

#ifdef DEBUG_PRINT_MP
	std::cout<<"[MP]: Will sleep now until we have linearized dynamics."<<std::endl;
#endif //DEBUG_PRINT_MP

	std::unique_lock<std::mutex> waitLock(kCompletedMutex_);
	kCompletedCondition_.wait(waitLock, [this]{return kCompleted_.load() >= KMax_;});
	waitLock.unlock();
	workerTask_ = IDLE;
#ifdef DEBUG_PRINT_MP
	std::cout<<"[MP]: Woke up again, should have linearized dynamics now."<<std::endl;
#endif //DEBUG_PRINT_MP
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void iLQGMP<STATE_DIM, CONTROL_DIM, SCALAR>::computeLinearizedDynamicsWorker(size_t threadId)
{
	while(true)
	{
		size_t k = kTaken_++;

		if (k >= KMax_)
		{
			//kCompleted_++;
			if (kCompleted_.load() >= KMax_)
				kCompletedCondition_.notify_all();
			return;
		}

#ifdef DEBUG_PRINT_MP
		if ((k+1)%100 == 0)
			std::cout<<"[Thread "<<threadId<<"]: Linearizing for index k "<<KMax_ - k - 1<<std::endl;
#endif

		this->computeLinearizedDynamics(threadId, KMax_-k-1); // linearize backwards

		kCompleted_++;
	}
}



template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void iLQGMP<STATE_DIM, CONTROL_DIM, SCALAR>::computeQuadraticCostsAroundTrajectory()
{
	Eigen::setNbThreads(1); // disable Eigen multi-threading

	kTaken_ = 0;
	kCompleted_ = 0;
	KMax_ = this->K_;

#ifdef DEBUG_PRINT_MP
	std::cout<<"[MP]: Waking up workers to do cost computation."<<std::endl;
#endif //DEBUG_PRINT_MP
	workerTask_ = COMPUTE_COST;
	workerWakeUpCondition_.notify_all();

#ifdef DEBUG_PRINT_MP
	std::cout<<"[MP]: Will sleep now until we have cost."<<std::endl;
#endif //DEBUG_PRINT_MP

	std::unique_lock<std::mutex> waitLock(kCompletedMutex_);
	kCompletedCondition_.wait(waitLock, [this]{return kCompleted_.load() >= KMax_;});
	waitLock.unlock();
	workerTask_ = IDLE;
#ifdef DEBUG_PRINT_MP
	std::cout<<"[MP]: Woke up again, should have cost now."<<std::endl;
#endif //DEBUG_PRINT_MP
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void iLQGMP<STATE_DIM, CONTROL_DIM, SCALAR>::computeQuadraticCostsWorker(size_t threadId)
{
	while(true)
	{
		size_t k = kTaken_++;

		if (k >= KMax_)
		{
			//kCompleted_++;
			if (kCompleted_.load() >= KMax_)
				kCompletedCondition_.notify_all();
			return;
		}

#ifdef DEBUG_PRINT_MP
		if ((k+1)%100 == 0)
			std::cout<<"[Thread "<<threadId<<"]: Quadratizing cost for index k "<<KMax_ - k - 1<<std::endl;
#endif

		this->computeQuadraticCosts(threadId, KMax_ - k - 1); // compute cost backwards

		kCompleted_++;
	}
}



template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void iLQGMP<STATE_DIM, CONTROL_DIM, SCALAR>::computeLQProblemWorker(size_t threadId)
{
	while(true)
	{
		size_t k = kTaken_++;

		if (k >= KMax_)
		{
			//kCompleted_++;
			if (kCompleted_.load() >= KMax_)
				kCompletedCondition_.notify_all();
			return;
		}

#ifdef DEBUG_PRINT_MP
		if ((k+1)%100 == 0)
			std::cout<<"[Thread "<<threadId<<"]: Building LQ problem for index k "<<KMax_ - k - 1<<std::endl;
#endif

		this->computeQuadraticCosts(threadId, KMax_-k-1); // compute cost backwards
		this->computeLinearizedDynamics(threadId, KMax_-k-1); // linearize backwards

		kCompleted_++;
	}
}



}
}
