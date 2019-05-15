/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <cmath>
#include <functional>

#include "SensitivityIntegratorCT.h"

#include <ct/optcon/costfunction/CostFunctionQuadratic.hpp>

#include <ct/optcon/dms/dms_core/DmsDimensions.h>
#include <ct/optcon/dms/dms_core/OptVectorDms.h>
#include <ct/optcon/dms/dms_core/ControllerDms.h>

namespace ct {
namespace optcon {

/**
 * @ingroup    DMS
 *
 * @brief      This class performs the state and the sensitivity integration on
 *             a shot
 *
 * @tparam     STATE_DIM    The state dimension
 * @tparam     CONTROL_DIM  The control dimension
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class ShotContainer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef DmsDimensions<STATE_DIM, CONTROL_DIM, SCALAR> DIMENSIONS;

    typedef typename DIMENSIONS::state_vector_t state_vector_t;
    typedef typename DIMENSIONS::control_vector_t control_vector_t;
    typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
    typedef typename DIMENSIONS::control_vector_array_t control_vector_array_t;
    typedef typename DIMENSIONS::time_array_t time_array_t;

    typedef typename DIMENSIONS::state_matrix_t state_matrix_t;
    typedef typename DIMENSIONS::state_control_matrix_t state_control_matrix_t;

    typedef typename DIMENSIONS::state_matrix_array_t state_matrix_array_t;
    typedef typename DIMENSIONS::state_control_matrix_array_t state_control_matrix_array_t;

    ShotContainer() = delete;

    /**
	 * @brief      Custom constructor
	 *
	 * @param[in]  controlledSystem  The nonlinear system
	 * @param[in]  linearSystem      The linearized system
	 * @param[in]  costFct           The costfunction
	 * @param[in]  w                 The optimization vector
	 * @param[in]  controlSpliner    The control input spliner
	 * @param[in]  timeGrid          The timegrid 
	 * @param[in]  shotNr            The shot number
	 * @param[in]  settings          The dms settings
	 */
    ShotContainer(std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>> controlledSystem,
        std::shared_ptr<ct::core::LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>> linearSystem,
        std::shared_ptr<ct::optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>> costFct,
        std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>> w,
        std::shared_ptr<SplinerBase<control_vector_t, SCALAR>> controlSpliner,
        std::shared_ptr<tpl::TimeGrid<SCALAR>> timeGrid,
        size_t shotNr,
        DmsSettings settings,
        size_t nIntegrationSteps)
        : controlledSystem_(controlledSystem),
          linearSystem_(linearSystem),
          costFct_(costFct),
          w_(w),
          controlSpliner_(controlSpliner),
          timeGrid_(timeGrid),
          shotNr_(shotNr),
          settings_(settings),
          integrationCount_(0),
          costIntegrationCount_(0),
          sensIntegrationCount_(0),
          costSensIntegrationCount_(0),
          cost_(SCALAR(0.0)),
          discreteQ_(state_vector_t::Zero()),
          discreteR_(control_vector_t::Zero()),
          discreteRNext_(control_vector_t::Zero())
    {
        if (shotNr_ >= settings.N_)
            throw std::runtime_error("Dms Shot Integrator: shot index >= settings.N_ - check your settings.");

        switch (settings_.integrationType_)
        {
            case DmsSettings::EULER:
            {
                integratorCT_ = std::allocate_shared<SensitivityIntegratorCT<STATE_DIM, CONTROL_DIM, SCALAR>,
                    Eigen::aligned_allocator<SensitivityIntegratorCT<STATE_DIM, CONTROL_DIM, SCALAR>>>(
                    Eigen::aligned_allocator<SensitivityIntegratorCT<STATE_DIM, CONTROL_DIM, SCALAR>>(),
                    controlledSystem_, core::EULERCT);
                break;
            }
            case DmsSettings::RK4:
            {
                integratorCT_ = std::allocate_shared<SensitivityIntegratorCT<STATE_DIM, CONTROL_DIM, SCALAR>,
                    Eigen::aligned_allocator<SensitivityIntegratorCT<STATE_DIM, CONTROL_DIM, SCALAR>>>(
                    Eigen::aligned_allocator<SensitivityIntegratorCT<STATE_DIM, CONTROL_DIM, SCALAR>>(),
                    controlledSystem_, core::RK4CT);
                break;
            }
            case DmsSettings::RK5:
            {
                throw std::runtime_error("Currently we do not support adaptive integrators in dms");
            }

            default:
            {
                std::cerr << "... ERROR: unknown integration type. Exiting" << std::endl;
                exit(0);
            }
        }

        tStart_ = timeGrid_->getShotStartTime(shotNr_);
        // SCALAR t_shot_end = timeGrid_->getShotEndTime(shotNr_);

        // +0.5 needed to avoid rounding errors from double to size_t
        nSteps_ = nIntegrationSteps;
        // std::cout << "shotNr_: " << shotNr_ << "\t nSteps: " << nSteps_ << std::endl;

        integratorCT_->setLinearSystem(linearSystem_);

        if (settings_.costEvaluationType_ == DmsSettings::FULL)
            integratorCT_->setCostFunction(costFct_);
    }

    /**
	 * @brief      Performs the state integration between the shots
	 */
    void integrateShot()
    {
        if ((w_->getUpdateCount() != integrationCount_))
        {
            integrationCount_ = w_->getUpdateCount();
            state_vector_t initState = w_->getOptimizedState(shotNr_);
            integratorCT_->integrate(
                initState, tStart_, nSteps_, SCALAR(settings_.dt_sim_), stateSubsteps_, timeSubsteps_);
        }
    }

    void integrateCost()
    {
        if ((w_->getUpdateCount() != costIntegrationCount_))
        {
            costIntegrationCount_ = w_->getUpdateCount();
            integrateShot();
            cost_ = SCALAR(0.0);
            integratorCT_->integrateCost(cost_, tStart_, nSteps_, SCALAR(settings_.dt_sim_));
        }
    }

    /**
	 * @brief      Performs the state and the sensitivity integration between the shots
	 */
    void integrateSensitivities()
    {
        if ((w_->getUpdateCount() != sensIntegrationCount_))
        {
            sensIntegrationCount_ = w_->getUpdateCount();
            integrateShot();
            discreteA_.setIdentity();
            discreteB_.setZero();
            integratorCT_->linearize();
            integratorCT_->integrateSensitivityDX0(discreteA_, tStart_, nSteps_, SCALAR(settings_.dt_sim_));
            integratorCT_->integrateSensitivityDU0(discreteB_, tStart_, nSteps_, SCALAR(settings_.dt_sim_));

            if (settings_.splineType_ == DmsSettings::PIECEWISE_LINEAR)
            {
                discreteBNext_.setZero();
                integratorCT_->integrateSensitivityDUf(discreteBNext_, tStart_, nSteps_, SCALAR(settings_.dt_sim_));
            }
        }
    }

    void integrateCostSensitivities()
    {
        if ((w_->getUpdateCount() != costSensIntegrationCount_))
        {
            costSensIntegrationCount_ = w_->getUpdateCount();
            integrateSensitivities();
            discreteQ_.setZero();
            discreteR_.setZero();
            integratorCT_->integrateCostSensitivityDX0(discreteQ_, tStart_, nSteps_, SCALAR(settings_.dt_sim_));
            integratorCT_->integrateCostSensitivityDU0(discreteR_, tStart_, nSteps_, SCALAR(settings_.dt_sim_));

            if (settings_.splineType_ == DmsSettings::PIECEWISE_LINEAR)
            {
                discreteRNext_.setZero();
                integratorCT_->integrateCostSensitivityDUf(discreteRNext_, tStart_, nSteps_, SCALAR(settings_.dt_sim_));
            }
        }
    }

    void reset()
    {
        integratorCT_->clearStates();
        integratorCT_->clearSensitivities();
        integratorCT_->clearLinearization();
    }

    /**
	 * @brief      Returns the integrated state
	 *
	 * @return     The integrated state
	 */
    const state_vector_t& getStateIntegrated() { return stateSubsteps_.back(); }
    /**
	 * @brief      Returns the end time of the integration	
	 *
	 * @return     The end time of the integration.
	 */
    const SCALAR getIntegrationTimeFinal() { return timeSubsteps_.back(); }
    /**
	 * @brief      Returns the integrated ODE sensitivity with respect to the
	 *             discretized state s_i
	 *
	 * @return     The integrated sensitivity
	 */
    const state_matrix_t& getdXdSiIntegrated() { return discreteA_; }
    /**
	 * @brief      Returns the integrated ODE sensitivity with respect to the
	 *             discretized inputs q_i
	 *
	 * @return     The integrated sensitivity
	 */
    const state_control_matrix_t& getdXdQiIntegrated() { return discreteB_; }
    /**
	 * @brief      Returns the integrated ODE sensitivity with respect to the
	 *             discretized inputs q_{i+1}
	 *
	 * @return     The integrated sensitivity
	 */
    const state_control_matrix_t& getdXdQip1Integrated() { return discreteBNext_; }
    /**
	 * @brief      Returns the integrated ODE sensitivity with respect to the
	 *             time segments h_i
	 *
	 * @return     The integrated sensitivity
	 */
    // const state_vector_t getdXdHiIntegrated()
    // {
    // 	return dXdHi_history_.back();
    // }

    /**
	 * @brief      Gets the full integrated state trajectory.
	 *
	 * @return     The integrated state trajectory
	 */
    const state_vector_array_t& getXHistory() const { return stateSubsteps_; }
    /**
	 * @brief      Returns the control input trajectory used during the state integration
	 *
	 * @return     The control trajectory
	 */
    const control_vector_array_t& getUHistory()
    {
        inputSubsteps_.clear();
        for (size_t t = 0; t < timeSubsteps_.size(); ++t)
        {
            inputSubsteps_.push_back(controlSpliner_->evalSpline(timeSubsteps_[t], shotNr_));
        }
        return inputSubsteps_;
    }

    /**
	 * @brief      Returns the time trajectory used during the integration
	 *
	 * @return     The time trajectory
	 */
    const time_array_t& getTHistory() const { return timeSubsteps_; }
    /**
	 * @brief      Gets the cost integrated.
	 *
	 * @return     The integrated cost.
	 */
    const SCALAR getCostIntegrated() const { return cost_; }
    /**
	 * @brief      Returns the cost gradient with respect to s_i integrated over
	 *             the shot
	 *
	 * @return     The cost gradient
	 */
    const state_vector_t& getdLdSiIntegrated() const { return discreteQ_; }
    /**
	 * @brief      Returns the cost gradient with respect to q_i integrated over
	 *             the shot
	 *
	 * @return     The cost gradient
	 */
    const control_vector_t& getdLdQiIntegrated() const { return discreteR_; }
    /**
	 * @brief      Returns to cost gradient with respect to q_{i+1} integrated
	 *             over the shot
	 *
	 * @return     The cost gradient
	 */
    const control_vector_t& getdLdQip1Integrated() const { return discreteRNext_; }
    /**
	 * @brief      Returns to cost gradient with respect to h_i integrated over
	 *             the shot
	 *
	 * @return     The cost gradient
	 */
    // const double getdLdHiIntegrated() const
    // {
    // 	return costGradientHi_;
    // }


private:
    std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>> controlledSystem_;
    std::shared_ptr<ct::core::LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>> linearSystem_;
    std::shared_ptr<ct::optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>> costFct_;
    std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>> w_;
    std::shared_ptr<SplinerBase<control_vector_t, SCALAR>> controlSpliner_;
    std::shared_ptr<tpl::TimeGrid<SCALAR>> timeGrid_;

    const size_t shotNr_;
    const DmsSettings settings_;

    size_t integrationCount_;
    size_t costIntegrationCount_;
    size_t sensIntegrationCount_;
    size_t costSensIntegrationCount_;

    // Integrated trajectories
    state_vector_array_t stateSubsteps_;
    control_vector_array_t inputSubsteps_;
    time_array_t timeSubsteps_;

    //Sensitivity Trajectories
    state_matrix_t discreteA_;
    state_control_matrix_t discreteB_;
    state_control_matrix_t discreteBNext_;

    //Cost and cost gradient
    SCALAR cost_;
    state_vector_t discreteQ_;
    control_vector_t discreteR_;
    control_vector_t discreteRNext_;

    std::shared_ptr<SensitivityIntegratorCT<STATE_DIM, CONTROL_DIM, SCALAR>> integratorCT_;
    size_t nSteps_;
    SCALAR tStart_;
};

}  // namespace optcon
}  // namespace ct
