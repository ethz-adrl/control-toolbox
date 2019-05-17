/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <omp.h>
#include <math.h>
#include <cmath>
#include <functional>

#include <ct/optcon/costfunction/CostFunctionQuadratic.hpp>

#include <ct/optcon/dms/dms_core/OptVectorDms.h>
#include <ct/optcon/dms/dms_core/ShotContainer.h>
#include <ct/optcon/nlp/DiscreteCostEvaluatorBase.h>


namespace ct {
namespace optcon {

/**
 * @ingroup    DMS
 *
 * @brief      Performs the full cost integration over the shots
 *
 * @tparam     STATE_DIM    The state dimension
 * @tparam     CONTROL_DIM  The input dimension
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class CostEvaluatorFull : public tpl::DiscreteCostEvaluatorBase<SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef DmsDimensions<STATE_DIM, CONTROL_DIM, SCALAR> DIMENSIONS;

    typedef typename DIMENSIONS::state_vector_t state_vector_t;
    typedef typename DIMENSIONS::control_vector_t control_vector_t;
    typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
    typedef typename DIMENSIONS::control_vector_array_t control_vector_array_t;
    typedef typename DIMENSIONS::time_array_t time_array_t;

    CostEvaluatorFull() = delete;

    /**
	 * @brief      Custom constructor
	 *
	 * @param[in]  costFct         The cost function
	 * @param[in]  w               The optimization vector
	 * @param[in]  controlSpliner  The control spliner
	 * @param[in]  shotInt         The shot number
	 * @param[in]  settings        The dms settings
	 */
    CostEvaluatorFull(std::shared_ptr<ct::optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>> costFct,
        std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>> w,
        std::shared_ptr<SplinerBase<control_vector_t, SCALAR>> controlSpliner,
        std::vector<std::shared_ptr<ShotContainer<STATE_DIM, CONTROL_DIM, SCALAR>>> shotInt,
        DmsSettings settings)
        : costFct_(costFct), w_(w), controlSpliner_(controlSpliner), shotContainers_(shotInt), settings_(settings)
    {
    }

    /**
	 * @brief      The destructor.
	 */
    ~CostEvaluatorFull() override = default;

    SCALAR eval() override
    {
        SCALAR cost = SCALAR(0.0);

#pragma omp parallel for num_threads(settings_.nThreads_)
        for (auto shotContainer = shotContainers_.begin(); shotContainer < shotContainers_.end(); ++shotContainer)
        {
            (*shotContainer)->integrateCost();
        }

        for (auto shotContainer : shotContainers_)
            cost += shotContainer->getCostIntegrated();

        costFct_->setCurrentStateAndControl(w_->getOptimizedState(settings_.N_), control_vector_t::Zero());
        cost += costFct_->evaluateTerminal();
        return cost;
    }

    void evalGradient(size_t grad_length, Eigen::Map<Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>>& grad) override
    {
        grad.setZero();

        assert(shotContainers_.size() == settings_.N_);

// go through all shots, integrate the state trajectories and evaluate cost accordingly
// intermediate costs
#pragma omp parallel for num_threads(settings_.nThreads_)
        for (auto shotContainer = shotContainers_.begin(); shotContainer < shotContainers_.end(); ++shotContainer)
        {
            (*shotContainer)->integrateCostSensitivities();
        }

        for (size_t shotNr = 0; shotNr < shotContainers_.size(); ++shotNr)
        {
            switch (settings_.splineType_)
            {
                case DmsSettings::ZERO_ORDER_HOLD:
                {
                    grad.segment(w_->getStateIndex(shotNr), STATE_DIM) += shotContainers_[shotNr]->getdLdSiIntegrated();
                    grad.segment(w_->getControlIndex(shotNr), CONTROL_DIM) +=
                        shotContainers_[shotNr]->getdLdQiIntegrated();
                    break;
                }
                case DmsSettings::PIECEWISE_LINEAR:
                {
                    grad.segment(w_->getStateIndex(shotNr), STATE_DIM) += shotContainers_[shotNr]->getdLdSiIntegrated();
                    grad.segment(w_->getControlIndex(shotNr), CONTROL_DIM) +=
                        shotContainers_[shotNr]->getdLdQiIntegrated();
                    grad.segment(w_->getControlIndex(shotNr + 1), CONTROL_DIM) +=
                        shotContainers_[shotNr]->getdLdQip1Integrated();
                    break;
                }
                default:
                    throw(std::runtime_error(
                        " cost gradient not yet implemented for this type of interpolation. Exiting"));
            }

            // H-part.
            // if(settings_.objectiveType_ == DmsSettings::OPTIMIZE_GRID)
            // {
            // 	costFct_->setCurrentStateAndControl(shotContainers_[shotNr]->getStateIntegrated(),
            // 										controlSpliner_->evalSpline(shotContainers_[shotNr]->getIntegrationTimeFinal(), shotNr));
            // 	grad(w_->getTimeSegmentIndex(shotNr)) = costFct_->evaluateIntermediate() + shotContainers_[shotNr]->getdLdHiIntegrated();
            // }
        }

        /* gradient of terminal cost */
        costFct_->setCurrentStateAndControl(w_->getOptimizedState(settings_.N_), control_vector_t::Zero());
        grad.segment(w_->getStateIndex(settings_.N_), STATE_DIM) +=
            costFct_->stateDerivativeTerminal();  // * dXdSi.back();
    }

private:
    std::shared_ptr<ct::optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>> costFct_;
    std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>> w_;
    std::shared_ptr<SplinerBase<control_vector_t, SCALAR>> controlSpliner_;
    std::vector<std::shared_ptr<ShotContainer<STATE_DIM, CONTROL_DIM, SCALAR>>> shotContainers_;

    const DmsSettings settings_;
};

}  // namespace optcon
}  // namespace ct
