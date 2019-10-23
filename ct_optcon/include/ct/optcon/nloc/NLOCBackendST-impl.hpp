/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
NLOCBackendST<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::NLOCBackendST(
    const OptConProblem_t& optConProblem,
    const NLOptConSettings& settings)
    : Base(optConProblem, settings)
{
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
NLOCBackendST<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::NLOCBackendST(
    const OptConProblem_t& optConProblem,
    const std::string& settingsFile,
    bool verbose,
    const std::string& ns)
    : Base(optConProblem, settingsFile, verbose, ns)
{
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
NLOCBackendST<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::~NLOCBackendST()
{
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
void NLOCBackendST<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::computeLQApproximation(size_t firstIndex,
    size_t lastIndex)
{
    if (lastIndex == static_cast<size_t>(this->K_) - 1)
        this->initializeCostToGo();

    for (size_t k = firstIndex; k <= lastIndex; k++)
    {
        this->executeLQApproximation(this->settings_.nThreads, k);

        if (this->generalConstraints_[this->settings_.nThreads] != nullptr)
            this->computeLinearizedConstraints(this->settings_.nThreads, k);
    }
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
void NLOCBackendST<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::rolloutShots(size_t firstIndex,
    size_t lastIndex)
{
    for (size_t k = firstIndex; k <= lastIndex; k = k + this->getNumStepsPerShot())
    {
        // rollout the shot
        this->rolloutSingleShot(this->settings_.nThreads, k, this->u_ff_, this->x_, this->x_ref_lqr_, this->xShot_,
            *this->substepsX_, *this->substepsU_);

        this->computeSingleDefect(k, this->x_, this->xShot_, this->d_);
    }
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
SCALAR NLOCBackendST<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::performLineSearch()
{
    // we start with extrapolation
    double alpha = this->settings_.lineSearchSettings.alpha_0;
    double alphaBest = 0.0;
    size_t iterations = 0;

    this->lx_norm_ = 0.0;
    this->lu_norm_ = 0.0;


    while (iterations < this->settings_.lineSearchSettings.maxIterations)
    {
        if (this->settings_.lineSearchSettings.debugPrint)
            std::cout << "[LineSearch]: Iteration: " << iterations << ", try alpha: " << alpha << " out of maximum "
                      << this->settings_.lineSearchSettings.maxIterations << " iterations. " << std::endl;

        iterations++;

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


        this->executeLineSearch(this->settings_.nThreads, alpha, x_search, x_shot_search, defects_recorded, u_recorded,
            intermediateCost, finalCost, defectNorm, e_box_norm, e_gen_norm, *substepsX, *substepsU);

        // compute new merit and check for step acceptance
        bool stepAccepted =
            this->acceptStep(alpha, intermediateCost, finalCost, defectNorm, e_box_norm, e_gen_norm, this->lowestCost_, cost);

        // catch the case that a rollout might be unstable
        if (!stepAccepted)
        {
            if (this->settings_.lineSearchSettings.debugPrint)
            {
                std::cout << "[LineSearch]: No better cost/merit found at alpha " << alpha << ":" << std::endl;
                std::cout << "[LineSearch]: Cost:\t" << intermediateCost + finalCost << std::endl;
                std::cout << "[LineSearch]: Defect:\t" << defectNorm << std::endl;
                std::cout << "[LineSearch]: err box constr:\t" + std::to_string(e_box_norm) << std::endl;
                std::cout << "[LineSearch]: err gen constr:\t" + std::to_string(e_gen_norm) << std::endl;
                std::cout << "[LineSearch]: Merit:\t" << cost << std::endl;
            }

            // compute new alpha
            alpha = alpha * this->settings_.lineSearchSettings.n_alpha;
        }
        else
        {
            // step accepted

            if (this->settings_.lineSearchSettings.debugPrint)
            {
                std::cout << "Lower cost/merit found at alpha: " << alpha << ":" << std::endl;
                std::cout << "[LineSearch]: Cost:\t" << intermediateCost + finalCost << std::endl;
                std::cout << "[LineSearch]: Defect:\t" << defectNorm << std::endl;
                std::cout << "[LineSearch]: err box constr:\t" + std::to_string(e_box_norm) << std::endl;
                std::cout << "[LineSearch]: err gen constr:\t" + std::to_string(e_gen_norm) << std::endl;
                std::cout << "[LineSearch]: Merit:\t" << cost << std::endl;
            }

            // compute update norms separately, as they are typically different from pure lqoc solver updates
            this->lu_norm_ =
                this->template computeDiscreteArrayNorm<ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>, 2>(
                    u_recorded, this->u_ff_prev_);
            this->lx_norm_ = this->template computeDiscreteArrayNorm<ct::core::StateVectorArray<STATE_DIM, SCALAR>, 2>(
                x_search, this->x_prev_);

            alphaBest = alpha;
            this->intermediateCostBest_ = intermediateCost;
            this->finalCostBest_ = finalCost;
            this->d_norm_ = defectNorm;
            this->e_box_norm_ = e_box_norm;
            this->e_gen_norm_ = e_gen_norm;
            this->x_prev_ = x_search;
            this->lowestCost_ = cost;
            this->x_.swap(x_search);
            this->xShot_.swap(x_shot_search);
            this->u_ff_.swap(u_recorded);
            this->d_.swap(defects_recorded);
            this->substepsX_ = substepsX;
            this->substepsU_ = substepsU;
            break;
        }
    }  // end while

    return alphaBest;
}


}  // namespace optcon
}  // namespace ct
