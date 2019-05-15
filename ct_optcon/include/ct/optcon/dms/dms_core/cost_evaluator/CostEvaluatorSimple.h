/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <omp.h>
#include <math.h>
#include <cmath>

#include <ct/optcon/costfunction/CostFunctionQuadratic.hpp>

#include <ct/optcon/dms/dms_core/OptVectorDms.h>
#include <ct/optcon/dms/dms_core/spline/SplinerBase.h>
#include <ct/optcon/nlp/DiscreteCostEvaluatorBase.h>


namespace ct {
namespace optcon {


/**
 * @ingroup    DMS
 *
 * @brief      Evaluates the cost at the shots and performs some interpolation
 *             in between
 *
 * @tparam     STATE_DIM    The state dimension
 * @tparam     CONTROL_DIM  The control dimension
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class CostEvaluatorSimple : public tpl::DiscreteCostEvaluatorBase<SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef DmsDimensions<STATE_DIM, CONTROL_DIM, SCALAR> DIMENSIONS;

    typedef typename DIMENSIONS::state_vector_t state_vector_t;
    typedef typename DIMENSIONS::control_vector_t control_vector_t;

    CostEvaluatorSimple() = delete;

    /**
   * @brief      Custom constructor
   *
   * @param[in]  costFct   The cost function
   * @param[in]  w         The optimization variables
   * @param[in]  timeGrid  The time grid
   * @param[in]  settings  The dms settings
   */
    CostEvaluatorSimple(std::shared_ptr<ct::optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>> costFct,
        std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>> w,
        std::shared_ptr<tpl::TimeGrid<SCALAR>> timeGrid,
        DmsSettings settings)
        : costFct_(costFct), w_(w), timeGrid_(timeGrid), settings_(settings)
    {
        phi_.resize(settings_.N_ + 1);
        // phi_diff_h_ = Eigen::VectorXd::Ones(settings_.N_+1, 1);
        updatePhi();
    }

    ~CostEvaluatorSimple() override = default;
    SCALAR eval() override;

    void evalGradient(size_t grad_length, Eigen::Map<Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>>& grad) override;

private:
    /**
   * @brief      Updates the weights for the cost interpolation
   */
    void updatePhi();

    std::shared_ptr<ct::optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>> costFct_;
    std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>> w_;
    std::shared_ptr<tpl::TimeGrid<SCALAR>> timeGrid_;
    const DmsSettings settings_;
    Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> phi_; /* the summation weights */
    // Eigen::VectorXd phi_diff_h_; /* the summation weights for derivative w.r.t h */
};


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
SCALAR CostEvaluatorSimple<STATE_DIM, CONTROL_DIM, SCALAR>::eval()
{
    // updatePhi();
    SCALAR cost = SCALAR(0.0);

    for (size_t i = 0; i < settings_.N_ + 1; ++i)
    {
        costFct_->setCurrentStateAndControl(
            w_->getOptimizedState(i), w_->getOptimizedControl(i), timeGrid_->getShotStartTime(i));
        cost += phi_(i) * costFct_->evaluateIntermediate();
    }

    costFct_->setCurrentStateAndControl(w_->getOptimizedState(settings_.N_), control_vector_t::Zero());
    cost += costFct_->evaluateTerminal();
    assert(cost == cost);
    return cost;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void CostEvaluatorSimple<STATE_DIM, CONTROL_DIM, SCALAR>::evalGradient(size_t grad_length,
    Eigen::Map<Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>>& grad)
{
    // updatePhi();
    grad.setZero();
    for (size_t i = 0; i < settings_.N_ + 1; ++i)
    {
        costFct_->setCurrentStateAndControl(
            w_->getOptimizedState(i), w_->getOptimizedControl(i), timeGrid_->getShotStartTime(i));
        grad.segment(w_->getStateIndex(i), STATE_DIM) += phi_(i) * costFct_->stateDerivativeIntermediate();
        grad.segment(w_->getControlIndex(i), CONTROL_DIM) += phi_(i) * costFct_->controlDerivativeIntermediate();
    }

    /* gradient of terminal cost */
    costFct_->setCurrentStateAndControl(w_->getOptimizedState(settings_.N_), control_vector_t::Zero());
    grad.segment(w_->getStateIndex(settings_.N_), STATE_DIM) += costFct_->stateDerivativeTerminal();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void CostEvaluatorSimple<STATE_DIM, CONTROL_DIM, SCALAR>::updatePhi()
{
    switch (settings_.splineType_)
    {
        case DmsSettings::ZERO_ORDER_HOLD:
        {
            for (size_t i = 0; i < settings_.N_; i++)
                phi_(i) = (timeGrid_->getShotDuration(i));

            phi_(settings_.N_) = SCALAR(0.0);
            break;
        }
        case DmsSettings::PIECEWISE_LINEAR:
        {
            phi_(0) = SCALAR(0.5) * (timeGrid_->getShotDuration(0));

            for (size_t i = 1; i < settings_.N_; i++)
                phi_(i) = SCALAR(0.5) * (timeGrid_->getShotEndTime(i) - timeGrid_->getShotStartTime(i - 1));

            phi_(settings_.N_) = SCALAR(0.5) * (timeGrid_->getShotDuration(settings_.N_));
            break;
        }
        default:
            throw(std::runtime_error(" ERROR: Unknown spline-type in CostEvaluatorSimple - exiting."));
    }
}

}  // namespace optcon
}  // namespace ct
