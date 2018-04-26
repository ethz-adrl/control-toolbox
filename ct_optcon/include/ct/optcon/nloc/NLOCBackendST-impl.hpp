/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
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
    if (lastIndex == this->K_ - 1)
        this->initializeCostToGo();

    for (size_t k = firstIndex; k <= lastIndex; k++)
    {
        this->computeLinearizedDynamics(this->settings_.nThreads, k);

        this->computeQuadraticCosts(this->settings_.nThreads, k);

        if (this->generalConstraints_[this->settings_.nThreads] != nullptr)
            this->computeLinearizedConstraints(this->settings_.nThreads, k);
    }
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
void NLOCBackendST<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::rolloutShots(size_t firstIndex,
    size_t lastIndex)
{
    for (size_t k = firstIndex; k <= lastIndex; k = k + this->settings_.K_shot)
    {
        // rollout the shot
        this->rolloutSingleShot(this->settings_.nThreads, k, this->u_ff_, this->x_, this->x_, this->xShot_,
            *this->substepsX_, *this->substepsU_);

        this->computeSingleDefect(k, this->x_, this->xShot_, this->lqocProblem_->b_);
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

        // set init state
        x_search[0] = this->x_[0];


        switch (this->settings_.nlocp_algorithm)
        {
            case NLOptConSettings::NLOCP_ALGORITHM::GNMS:
            {
                this->executeLineSearchMultipleShooting(this->settings_.nThreads, alpha, this->lu_, this->lx_, x_search,
                    x_shot_search, defects_recorded, u_recorded, intermediateCost, finalCost, defectNorm, e_box_norm,
                    e_gen_norm, *substepsX, *substepsU);

                break;
            }
            case NLOptConSettings::NLOCP_ALGORITHM::ILQR:
            {
                defectNorm = 0.0;
                this->executeLineSearchSingleShooting(this->settings_.nThreads, alpha, x_search, u_recorded,
                    intermediateCost, finalCost, e_box_norm, e_gen_norm, *substepsX, *substepsU);
                break;
            }
            default:
                throw std::runtime_error("Algorithm type unknown in performLineSearch()!");
        }

        // compute merit
        cost = intermediateCost + finalCost + this->settings_.meritFunctionRho * defectNorm +
               this->settings_.meritFunctionRhoConstraints * (e_box_norm + e_gen_norm);

        // catch the case that a rollout might be unstable
        if (std::isnan(cost) ||
            cost >= this->lowestCost_)  // TODO: alternatively cost >= (this->lowestCost_*(1 - 1e-3*alpha)), study this
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
            // cost < this->lowestCost_ , better merit/cost found!

            if (this->settings_.lineSearchSettings.debugPrint)
            {
                std::cout << "Lower cost/merit found at alpha: " << alpha << ":" << std::endl;
                std::cout << "[LineSearch]: Cost:\t" << intermediateCost + finalCost << std::endl;
                std::cout << "[LineSearch]: Defect:\t" << defectNorm << std::endl;
                std::cout << "[LineSearch]: err box constr:\t" + std::to_string(e_box_norm) << std::endl;
                std::cout << "[LineSearch]: err gen constr:\t" + std::to_string(e_gen_norm) << std::endl;
                std::cout << "[LineSearch]: Merit:\t" << cost << std::endl;
            }


            if (this->settings_.printSummary)
            {
                this->lu_norm_ =
                    this->template computeDiscreteArrayNorm<ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>, 2>(
                        u_recorded, this->u_ff_prev_);
                this->lx_norm_ =
                    this->template computeDiscreteArrayNorm<ct::core::StateVectorArray<STATE_DIM, SCALAR>, 2>(
                        x_search, this->x_prev_);
            }
            else
            {
#ifdef MATLAB
                this->lu_norm_ =
                    this->template computeDiscreteArrayNorm<ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>, 2>(
                        u_recorded, this->u_ff_prev_);
                this->lx_norm_ =
                    this->template computeDiscreteArrayNorm<ct::core::StateVectorArray<STATE_DIM, SCALAR>, 2>(
                        x_search, this->x_prev_);
#endif
            }

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
            this->lqocProblem_->b_.swap(defects_recorded);
            this->substepsX_ = substepsX;
            this->substepsU_ = substepsU;
            break;
        }
    }  // end while

    return alphaBest;
}


}  // namespace optcon
}  // namespace ct
