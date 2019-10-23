/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/


#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
NLOCBackendMP<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::NLOCBackendMP(
    const OptConProblem_t& optConProblem,
    const NLOptConSettings& settings)
    : Base(optConProblem, settings)
{
    startupRoutine();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
NLOCBackendMP<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::NLOCBackendMP(
    const OptConProblem_t& optConProblem,
    const std::string& settingsFile,
    bool verbose,
    const std::string& ns)
    : Base(optConProblem, settingsFile, verbose, ns)
{
    startupRoutine();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
NLOCBackendMP<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::~NLOCBackendMP()
{
    shutdownRoutine();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
void NLOCBackendMP<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::startupRoutine()
{
    launchWorkerThreads();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
void NLOCBackendMP<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::shutdownRoutine()
{
    workersActive_ = false;
    workerTask_ = SHUTDOWN;
    workerWakeUpCondition_.notify_all();

#ifdef DEBUG_PRINT_MP
    std::cout << "Shutting down workers" << std::endl;
#endif  // DEBUG_PRINT_MP

    for (size_t i = 0; i < workerThreads_.size(); i++)
    {
        workerThreads_[i].join();
    }

#ifdef DEBUG_PRINT_MP
    std::cout << "All workers shut down" << std::endl;
#endif  // DEBUG_PRINT_MP
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
void NLOCBackendMP<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::threadWork(size_t threadId)
{
#ifdef DEBUG_PRINT_MP
    printString("[Thread " + std::to_string(threadId) + "]: launched");
#endif  // DEBUG_PRINT_MP


    // local variables
    int workerTask_local = IDLE;
    size_t uniqueProcessID = 0;
    size_t iteration_local = this->iteration_;
    size_t lqpCounter_local = this->lqpCounter_;


    while (workersActive_)
    {
        workerTask_local = workerTask_.load();
        iteration_local = this->iteration_;
        lqpCounter_local = this->lqpCounter_;

#ifdef DEBUG_PRINT_MP
        printString("[Thread " + std::to_string(threadId) + "]: previous procId: " + std::to_string(uniqueProcessID) +
                    ", current procId: " +
                    std::to_string(generateUniqueProcessID(iteration_local, (int)workerTask_local, lqpCounter_local)));
#endif


        /*!
         * We want to put the worker to sleep if
         * - the workerTask_ is IDLE
         * - or we are finished but workerTask_ is not yet reset, thus the process ID is still the same
         * */
        if (workerTask_local == IDLE ||
            uniqueProcessID == generateUniqueProcessID(iteration_local, (int)workerTask_local, lqpCounter_local))
        {
#ifdef DEBUG_PRINT_MP
            printString("[Thread " + std::to_string(threadId) + "]: going to sleep !");
#endif

            // sleep until the state is not IDLE any more and we have a different process ID than before
            std::unique_lock<std::mutex> waitLock(workerWakeUpMutex_);
            while (workerTask_ == IDLE || (uniqueProcessID == generateUniqueProcessID(this->iteration_,
                                                                  (int)workerTask_.load(), this->lqpCounter_)))
            {
                workerWakeUpCondition_.wait(waitLock);
            }
            waitLock.unlock();

            workerTask_local = workerTask_.load();
            iteration_local = this->iteration_;
            lqpCounter_local = this->lqpCounter_;

#ifdef DEBUG_PRINT_MP
            printString("[Thread " + std::to_string(threadId) + "]: woke up !");
#endif  // DEBUG_PRINT_MP
        }

        if (!workersActive_)
        {
#ifdef DEBUG_PRINT_MP
            printString("Breaking - workers are not active !");
#endif  // DEBUG_PRINT_MP
            break;
        }


        switch (workerTask_local)
        {
            case LINE_SEARCH:
            {
#ifdef DEBUG_PRINT_MP
                printString("[Thread " + std::to_string(threadId) + "]: now busy with LINE_SEARCH !");
#endif  // DEBUG_PRINT_MP
                lineSearchWorker(threadId);
                uniqueProcessID = generateUniqueProcessID(iteration_local, LINE_SEARCH, lqpCounter_local);
                break;
            }
            case ROLLOUT_SHOTS:
            {
#ifdef DEBUG_PRINT_MP
                printString("[Thread " + std::to_string(threadId) + "]: now doing shot rollouts !");
#endif  // DEBUG_PRINT_MP
                rolloutShotWorker(threadId);
                uniqueProcessID = generateUniqueProcessID(iteration_local, ROLLOUT_SHOTS, lqpCounter_local);
                break;
            }
            case COMPUTE_LQ_PROBLEM:
            {
#ifdef DEBUG_PRINT_MP
                printString("[Thread " + std::to_string(threadId) + "]: now doing LQ approximation !");
#endif  // DEBUG_PRINT_MP
                computeLQProblemWorker(threadId);
                uniqueProcessID = generateUniqueProcessID(iteration_local, COMPUTE_LQ_PROBLEM, lqpCounter_local);
                break;
            }
            case SHUTDOWN:
            {
#ifdef DEBUG_PRINT_MP
                printString("[Thread " + std::to_string(threadId) + "]: now shutting down !");
#endif  // DEBUG_PRINT_MP
                return;
                break;
            }
            case IDLE:
            {
#ifdef DEBUG_PRINT_MP
                printString("[Thread " + std::to_string(threadId) + "]: is idle. Now going to sleep.");
#endif  // DEBUG_PRINT_MP
                break;
            }
            default:
            {
                printString("[Thread " + std::to_string(threadId) + "]: WARNING: Worker has unknown task !");
                break;
            }
        }
    }

#ifdef DEBUG_PRINT_MP
    printString("[Thread " + std::to_string(threadId) + "]: shut down.");
#endif  // DEBUG_PRINT_MP
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
void NLOCBackendMP<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::launchWorkerThreads()
{
#ifdef DEBUG_PRINT_MP
    printString("[MP]: Going to launch worker threads!");
    std::cout << workersActive_.load() << std::endl;
#endif  //DEBUG_PRINT_MP

    workersActive_ = true;
    workerTask_ = IDLE;

    for (int i = 0; i < (int)this->settings_.nThreads; i++)
    {
        workerThreads_.push_back(std::thread(&NLOCBackendMP::threadWork, this, i));
    }
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
void NLOCBackendMP<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::computeLQApproximation(size_t firstIndex,
    size_t lastIndex)
{
    // fill terminal cost
    if (lastIndex == (static_cast<size_t>(this->K_) - 1))
        this->initializeCostToGo();

    /*
	 * In special cases, this function may be called for a single index, e.g. for the unconstrained GNMS real-time iteration scheme.
	 * Then, don't wake up workers, but do single-threaded computation for that single index, and return.
	 */
    if (lastIndex == firstIndex)
    {
#ifdef DEBUG_PRINT_MP
        printString("[MP]: do single threaded LQ approximation for single index " + std::to_string(firstIndex) +
                    ". Not waking up workers.");
#endif  //DEBUG_PRINT_MP
        this->executeLQApproximation(this->settings_.nThreads, firstIndex);
        if (this->generalConstraints_[this->settings_.nThreads] != nullptr)
            this->computeLinearizedConstraints(this->settings_.nThreads, firstIndex);
        return;
    }

    /*
	 * In case of multiple points to perform LQ-approximation, start multi-threading:
	 */
    Eigen::setNbThreads(1);  // disable Eigen multi-threading
#ifdef DEBUG_PRINT_MP
    printString("[MP]: Restricting Eigen to " + std::to_string(Eigen::nbThreads()) + " threads.");
#endif  //DEBUG_PRINT_MP

    kTaken_ = 0;
    kCompleted_ = 0;
    KMax_ = lastIndex;
    KMin_ = firstIndex;

#ifdef DEBUG_PRINT_MP
    printString("[MP]: Waking up workers to do LQ approximation.");
#endif  //DEBUG_PRINT_MP
    workerTask_ = COMPUTE_LQ_PROBLEM;
    workerWakeUpCondition_.notify_all();

#ifdef DEBUG_PRINT_MP
    printString("[MP]: Will sleep now until done with LQ approximation.");
#endif  //DEBUG_PRINT_MP

    std::unique_lock<std::mutex> waitLock(kCompletedMutex_);
    kCompletedCondition_.wait(waitLock, [this] { return kCompleted_.load() > KMax_ - KMin_; });
    waitLock.unlock();
    workerTask_ = IDLE;
#ifdef DEBUG_PRINT_MP
    printString("[MP]: Woke up again, should have computed LQ approximation now.");
#endif  //DEBUG_PRINT_MP


    Eigen::setNbThreads(this->settings_.nThreadsEigen);  // restore Eigen multi-threading
#ifdef DEBUG_PRINT_MP
    printString("[MP]: Restoring " + std::to_string(Eigen::nbThreads()) + " Eigen threads.");
#endif  //DEBUG_PRINT_MP
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
void NLOCBackendMP<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::computeLQProblemWorker(size_t threadId)
{
    while (true)
    {
        const size_t k = kTaken_++;

        if (k > KMax_ - KMin_)
        {
#ifdef DEBUG_PRINT_MP
            if ((k + 1) % 100 == 0)
                printString("k > KMax_ - KMin_ with k =  " + std::to_string(k) + " and KMax_ is " +
                            std::to_string(KMax_) + " and KMin_ is " + std::to_string(KMin_));
#endif

            //kCompleted_++;
            if (kCompleted_.load() > KMax_ - KMin_)
                kCompletedCondition_.notify_all();
            return;
        }

#ifdef DEBUG_PRINT_MP
        if ((k + 1) % 100 == 0)
            printString("[Thread " + std::to_string(threadId) + "]: Building LQ problem for index k " +
                        std::to_string(KMax_ - k));
#endif

        this->executeLQApproximation(threadId, KMax_ - k);

        if (this->generalConstraints_[threadId] != nullptr)
            this->computeLinearizedConstraints(threadId, KMax_ - k);  // linearize constraints backwards

        kCompleted_++;
    }
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
void NLOCBackendMP<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::rolloutShots(size_t firstIndex,
    size_t lastIndex)
{
    /*!
	 * In special cases, this function may be called for a single index, e.g. for the unconstrained GNMS real-time iteration scheme.
	 * Then, don't wake up workers, but do single-threaded computation for that single index, and return.
	 */
    if (lastIndex == firstIndex)
    {
#ifdef DEBUG_PRINT_MP
        printString("[MP]: do single threaded shot rollout for single index " + std::to_string(firstIndex) +
                    ". Not waking up workers.");
#endif  //DEBUG_PRINT_MP

        this->rolloutSingleShot(this->settings_.nThreads, firstIndex, this->u_ff_, this->x_, this->x_ref_lqr_,
            this->xShot_, *this->substepsX_, *this->substepsU_);

        this->computeSingleDefect(firstIndex, this->x_, this->xShot_, this->d_);
        return;
    }

    /*!
	 * In case of multiple points to be linearized, start multi-threading:
	 */
    Eigen::setNbThreads(1);  // disable Eigen multi-threading
#ifdef DEBUG_PRINT_MP
    printString("[MP]: Restricting Eigen to " + std::to_string(Eigen::nbThreads()) + " threads.");
#endif  //DEBUG_PRINT_MP


    kTaken_ = 0;
    kCompleted_ = 0;
    KMax_ = lastIndex;
    KMin_ = firstIndex;


#ifdef DEBUG_PRINT_MP
    std::cout << "[MP]: Waking up workers to do shot rollouts." << std::endl;
#endif  //DEBUG_PRINT_MP
    workerTask_ = ROLLOUT_SHOTS;
    workerWakeUpCondition_.notify_all();

#ifdef DEBUG_PRINT_MP
    std::cout << "[MP]: Will sleep now until we have rolled out all shots." << std::endl;
#endif  //DEBUG_PRINT_MP

    std::unique_lock<std::mutex> waitLock(kCompletedMutex_);
    kCompletedCondition_.wait(waitLock, [this] { return kCompleted_.load() > KMax_ - KMin_; });
    waitLock.unlock();
    workerTask_ = IDLE;
#ifdef DEBUG_PRINT_MP
    std::cout << "[MP]: Woke up again, should have rolled out all shots now." << std::endl;
#endif  //DEBUG_PRINT_MP

    Eigen::setNbThreads(this->settings_.nThreadsEigen);  // restore Eigen multi-threading
#ifdef DEBUG_PRINT_MP
    printString("[MP]: Restoring " + std::to_string(Eigen::nbThreads()) + " Eigen threads.");
#endif  //DEBUG_PRINT_MP
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
void NLOCBackendMP<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::rolloutShotWorker(size_t threadId)
{
    while (true)
    {
        size_t k = kTaken_++;

        if (k > KMax_ - KMin_)
        {
            if (kCompleted_.load() > KMax_ - KMin_)
                kCompletedCondition_.notify_all();
            return;
        }

        size_t kShot = (KMax_ - k);
        if (kShot % ((size_t)this->getNumStepsPerShot()) ==
            0)  //! only rollout when we're meeting the beginning of a shot
        {
#ifdef DEBUG_PRINT_MP
            if ((k + 1) % 100 == 0)
                printString("[Thread " + std::to_string(threadId) + "]: rolling out shot with index " +
                            std::to_string(KMax_ - k));
#endif

            this->rolloutSingleShot(threadId, kShot, this->u_ff_, this->x_, this->x_ref_lqr_, this->xShot_,
                *this->substepsX_, *this->substepsU_);

            this->computeSingleDefect(kShot, this->x_, this->xShot_, this->d_);
        }

        kCompleted_++;
    }
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
SCALAR NLOCBackendMP<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::performLineSearch()
{
    Eigen::setNbThreads(1);  // disable Eigen multi-threading

    alphaProcessed_.clear();
    alphaTaken_ = 0;
    alphaBestFound_ = false;
    alphaExpBest_ = this->settings_.lineSearchSettings.maxIterations;
    alphaExpMax_ = this->settings_.lineSearchSettings.maxIterations;
    alphaProcessed_.resize(this->settings_.lineSearchSettings.maxIterations, 0);
    lowestCostPrevious_ = this->lowestCost_;

#ifdef DEBUG_PRINT_MP
    std::cout << "[MP]: Waking up workers." << std::endl;
#endif  //DEBUG_PRINT_MP
    workerTask_ = LINE_SEARCH;
    workerWakeUpCondition_.notify_all();

#ifdef DEBUG_PRINT_MP
    std::cout << "[MP]: Will sleep now until done line search." << std::endl;
#endif  //DEBUG_PRINT_MP
    std::unique_lock<std::mutex> waitLock(alphaBestFoundMutex_);
    alphaBestFoundCondition_.wait(waitLock, [this] { return alphaBestFound_.load(); });
    waitLock.unlock();
    workerTask_ = IDLE;
#ifdef DEBUG_PRINT_MP
    std::cout << "[MP]: Woke up again, should have results now." << std::endl;
#endif  //DEBUG_PRINT_MP

    double alphaBest = 0.0;
    if (alphaExpBest_ != alphaExpMax_)
    {
        alphaBest = this->settings_.lineSearchSettings.alpha_0 *
                    std::pow(this->settings_.lineSearchSettings.n_alpha, alphaExpBest_);
    }

    Eigen::setNbThreads(this->settings_.nThreadsEigen);  // restore Eigen multi-threading
#ifdef DEBUG_PRINT_MP
    printString("[MP]: Restoring " + std::to_string(Eigen::nbThreads()) + " Eigen threads.");
#endif  //DEBUG_PRINT_MP

    // update norms, as they are typically different from the pure lqoc solver updates
    this->lu_norm_ = this->template computeDiscreteArrayNorm<ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>, 2>(
        this->u_ff_, this->u_ff_prev_);
    this->lx_norm_ = this->template computeDiscreteArrayNorm<ct::core::StateVectorArray<STATE_DIM, SCALAR>, 2>(
        this->x_, this->x_prev_);

    this->x_prev_ = this->x_;
    this->u_ff_prev_ = this->u_ff_;

    return alphaBest;

}  // end linesearch


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
void NLOCBackendMP<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::lineSearchWorker(size_t threadId)
{
    while (true)
    {
        size_t alphaExp = alphaTaken_++;

#ifdef DEBUG_PRINT_MP
        printString("[Thread " + std::to_string(threadId) + "]: Taking alpha index " + std::to_string(alphaExp));
#endif

        if (alphaExp >= alphaExpMax_ || alphaBestFound_)
        {
            return;
        }

        //! convert to real alpha
        double alpha =
            this->settings_.lineSearchSettings.alpha_0 * std::pow(this->settings_.lineSearchSettings.n_alpha, alphaExp);

        //! local variables
        SCALAR cost = std::numeric_limits<SCALAR>::max();
        SCALAR intermediateCost = std::numeric_limits<SCALAR>::max();
        SCALAR finalCost = std::numeric_limits<SCALAR>::max();
        SCALAR defectNorm = std::numeric_limits<SCALAR>::max();
        SCALAR e_box_norm = std::numeric_limits<SCALAR>::max();
        SCALAR e_gen_norm = std::numeric_limits<SCALAR>::max();
        ct::core::StateVectorArray<STATE_DIM, SCALAR> x_search(this->K_ + 1);
        ct::core::StateVectorArray<STATE_DIM, SCALAR> x_shot_search(this->K_ + 1);
        ct::core::StateVectorArray<STATE_DIM, SCALAR> defects_recorded(
            this->K_ + 1, ct::core::StateVector<STATE_DIM, SCALAR>::Zero());
        ct::core::ControlVectorArray<CONTROL_DIM, SCALAR> u_recorded(this->K_);
        typename Base::StateSubstepsPtr substepsX =
            typename Base::StateSubstepsPtr(new typename Base::StateSubsteps(this->K_ + 1));
        typename Base::ControlSubstepsPtr substepsU =
            typename Base::ControlSubstepsPtr(new typename Base::ControlSubsteps(this->K_ + 1));

        this->executeLineSearch(threadId, alpha, x_search, x_shot_search, defects_recorded, u_recorded,
            intermediateCost, finalCost, defectNorm, e_box_norm, e_gen_norm, *substepsX, *substepsU, &alphaBestFound_);

        lineSearchResultMutex_.lock();
        
        // check for step acceptance and get new merit/cost
        bool stepAccepted =
            this->acceptStep(alpha, intermediateCost, finalCost, defectNorm, e_box_norm, e_gen_norm, lowestCostPrevious_, cost);

        if (stepAccepted)
        {
            // make sure we do not alter an existing result
            if (alphaBestFound_)
            {
                lineSearchResultMutex_.unlock();
                break;
            }

            if (this->settings_.lineSearchSettings.debugPrint)
            {
                printString("[LineSearch, Thread " + std::to_string(threadId) +
                            "]: Lower cost/merit found at alpha:" + std::to_string(alpha));
                printString("[LineSearch]: Cost:\t" + std::to_string(intermediateCost + finalCost));
                printString("[LineSearch]: Defect:\t" + std::to_string(defectNorm));
                printString("[LineSearch]: err box constr:\t" + std::to_string(e_box_norm));
                printString("[LineSearch]: err gen constr:\t" + std::to_string(e_gen_norm));
                printString("[LineSearch]: Merit:\t" + std::to_string(cost));
            }

            alphaExpBest_ = alphaExp;
            this->intermediateCostBest_ = intermediateCost;
            this->finalCostBest_ = finalCost;
            this->d_norm_ = defectNorm;
            this->e_box_norm_ = e_box_norm;
            this->e_gen_norm_ = e_gen_norm;
            this->lowestCost_ = cost;
            this->x_.swap(x_search);
            this->xShot_.swap(x_shot_search);
            this->u_ff_.swap(u_recorded);
            this->d_.swap(defects_recorded);
            this->substepsX_ = substepsX;
            this->substepsU_ = substepsU;
        }
        else
        {
            if (this->settings_.lineSearchSettings.debugPrint)
            {
                if (!alphaBestFound_)
                {
                    printString("[LineSearch, Thread " + std::to_string(threadId) +
                                "]: NO lower cost/merit found at alpha:" + std::to_string(alpha));
                    printString("[LineSearch]: Cost:\t" + std::to_string(intermediateCost + finalCost));
                    printString("[LineSearch]: Defect:\t" + std::to_string(defectNorm));
                    printString("[LineSearch]: err box constr:\t" + std::to_string(e_box_norm));
                    printString("[LineSearch]: err gen constr:\t" + std::to_string(e_gen_norm));
                    printString("[LineSearch]: Merit:\t" + std::to_string(cost));
                }
                else
                    printString("[LineSearch, Thread " + std::to_string(threadId) +
                                "]: getting terminated. Best stepsize found by another thread.");
            }
        }

        alphaProcessed_[alphaExp] = 1;

        // we now check if all alphas prior to the best have been processed
        // this also covers the case that there is no better alpha
        bool allPreviousAlphasProcessed = true;
        for (size_t i = 0; i < alphaExpBest_; i++)
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

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
size_t NLOCBackendMP<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::generateUniqueProcessID(
    const size_t& iterateNo,
    const int workerState,
    const size_t resetCount)
{
    return (10e12 * (resetCount + 1) + 10e6 * (workerState + 1) + iterateNo + 1);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
void NLOCBackendMP<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::printString(const std::string& text)
{
    std::cout << text << std::endl;
}

}  // namespace optcon
}  // namespace ct
