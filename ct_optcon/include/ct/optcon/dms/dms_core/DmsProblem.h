/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/


#pragma once


#include <ct/optcon/problem/ContinuousOptConProblem.h>
#include <ct/optcon/dms/dms_core/DmsDimensions.h>
#include <ct/optcon/dms/dms_core/OptVectorDms.h>
#include <ct/optcon/dms/dms_core/ControllerDms.h>
#include <ct/optcon/dms/constraints/ConstraintsContainerDms.h>
#include <ct/optcon/dms/constraints/ConstraintDiscretizer.h>

#include <ct/optcon/dms/dms_core/cost_evaluator/CostEvaluatorSimple.h>
#include <ct/optcon/dms/dms_core/cost_evaluator/CostEvaluatorFull.h>
#include <ct/optcon/dms/dms_core/DmsSettings.h>

#include <ct/optcon/nlp/Nlp.h>

namespace ct {
namespace optcon {


/**
 * @ingroup    DMS
 *
 * @brief      This class sets up the DMS problem
 *
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class DmsProblem : public tpl::Nlp<SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef DmsDimensions<STATE_DIM, CONTROL_DIM, SCALAR> DIMENSIONS;
    typedef typename DIMENSIONS::state_vector_t state_vector_t;
    typedef typename DIMENSIONS::control_vector_array_t control_vector_array_t;
    typedef typename DIMENSIONS::control_vector_t control_vector_t;
    typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
    typedef typename DIMENSIONS::time_array_t time_array_t;

    typedef ContinuousOptConProblem<STATE_DIM, CONTROL_DIM, SCALAR> OptConProblem_t;


    /**
	 * @brief      Custom constructor, sets up the objects needed for the Dms
	 *             algorithm depending on the settings
	 *
	 * @param[in]  settings                 The Dms settings
	 * @param[in]  systemPtrs               The non linear systems
	 * @param[in]  linearPtrs               The linearized systems
	 * @param[in]  costPtrs                 The cost function
	 * @param[in]  inputBoxConstraints      The input box constraints
     * @param[in]  stateBoxConstraints      The state box constraints
	 * @param[in]  generaConstraints        The general constraints
	 * @param[in]  x0                       The initial state
	 */
    DmsProblem(DmsSettings settings,
        std::vector<typename OptConProblem_t::DynamicsPtr_t> systemPtrs,
        std::vector<typename OptConProblem_t::LinearPtr_t> linearPtrs,
        std::vector<typename OptConProblem_t::CostFunctionPtr_t> costPtrs,
        std::vector<typename OptConProblem_t::ConstraintPtr_t> inputBoxConstraints,
        std::vector<typename OptConProblem_t::ConstraintPtr_t> stateBoxConstraints,
        std::vector<typename OptConProblem_t::ConstraintPtr_t> generalConstraints,
        const state_vector_t& x0)
        : settings_(settings)
    {
        assert(systemPtrs.size() == settings_.N_);
        assert(linearPtrs.size() == settings_.N_);
        assert(costPtrs.size() == settings_.N_);
        settings_.parametersOk();

        timeGrid_ = std::shared_ptr<tpl::TimeGrid<SCALAR>>(new tpl::TimeGrid<SCALAR>(settings.N_, settings.T_));

        switch (settings_.splineType_)
        {
            case DmsSettings::ZERO_ORDER_HOLD:
            {
                controlSpliner_ = std::shared_ptr<ZeroOrderHoldSpliner<control_vector_t, SCALAR>>(
                    new ZeroOrderHoldSpliner<control_vector_t, SCALAR>(timeGrid_));
                break;
            }
            case DmsSettings::PIECEWISE_LINEAR:
            {
                controlSpliner_ = std::shared_ptr<LinearSpliner<control_vector_t, SCALAR>>(
                    new LinearSpliner<control_vector_t, SCALAR>(timeGrid_));
                break;
            }
            default:
                throw(std::runtime_error("Unknown spline type"));
        }


        size_t wLength = (settings.N_ + 1) * (STATE_DIM + CONTROL_DIM);
        if (settings_.objectiveType_ == DmsSettings::OPTIMIZE_GRID)
        {
            throw std::runtime_error("We do not support adaptive time gridding");
        }

        this->optVariables_ = std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>>(
            new OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>(wLength, settings));

        optVariablesDms_ = std::static_pointer_cast<OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>>(this->optVariables_);

        if (inputBoxConstraints.size() > 0 || stateBoxConstraints.size() > 0 || generalConstraints.size() > 0)
            discretizedConstraints_ = std::shared_ptr<ConstraintDiscretizer<STATE_DIM, CONTROL_DIM, SCALAR>>(
                new ConstraintDiscretizer<STATE_DIM, CONTROL_DIM, SCALAR>(
                    optVariablesDms_, controlSpliner_, timeGrid_, settings_.N_));

        if (inputBoxConstraints.size() > 0)
            discretizedConstraints_->setBoxConstraints(
                inputBoxConstraints
                    .front());  // at this point we do not distinguish between state and input box constraints

        if (stateBoxConstraints.size() > 0)
            discretizedConstraints_->setBoxConstraints(
                stateBoxConstraints
                    .front());  // at this point we do not distinguish between state and input box constraints

        if (generalConstraints.size() > 0)
            discretizedConstraints_->setGeneralConstraints(generalConstraints.front());


        for (size_t shotIdx = 0; shotIdx < settings_.N_; shotIdx++)
        {
            std::shared_ptr<ControllerDms<STATE_DIM, CONTROL_DIM, SCALAR>> newController(
                new ControllerDms<STATE_DIM, CONTROL_DIM, SCALAR>(controlSpliner_, shotIdx));
            systemPtrs[shotIdx]->setController(newController);
            linearPtrs[shotIdx]->setController(newController);

            size_t nIntegrationSteps =
                (timeGrid_->getShotEndTime(shotIdx) - timeGrid_->getShotStartTime(shotIdx)) / settings_.dt_sim_ + 0.5;

            shotContainers_.push_back(std::shared_ptr<ShotContainer<STATE_DIM, CONTROL_DIM, SCALAR>>(
                new ShotContainer<STATE_DIM, CONTROL_DIM, SCALAR>(systemPtrs[shotIdx], linearPtrs[shotIdx],
                    costPtrs[shotIdx], optVariablesDms_, controlSpliner_, timeGrid_, shotIdx, settings_,
                    nIntegrationSteps)));
        }

        switch (settings_.costEvaluationType_)
        {
            case DmsSettings::SIMPLE:
            {
                this->costEvaluator_ = std::shared_ptr<CostEvaluatorSimple<STATE_DIM, CONTROL_DIM, SCALAR>>(
                    new CostEvaluatorSimple<STATE_DIM, CONTROL_DIM, SCALAR>(
                        costPtrs.front(), optVariablesDms_, timeGrid_, settings_));
                break;
            }
            case DmsSettings::FULL:
            {
                this->costEvaluator_ = std::shared_ptr<CostEvaluatorFull<STATE_DIM, CONTROL_DIM, SCALAR>>(
                    new CostEvaluatorFull<STATE_DIM, CONTROL_DIM, SCALAR>(
                        costPtrs.front(), optVariablesDms_, controlSpliner_, shotContainers_, settings_));
                break;
            }
            default:
                throw(std::runtime_error("Unknown cost evaluation type"));
        }

        this->constraints_ = std::shared_ptr<ConstraintsContainerDms<STATE_DIM, CONTROL_DIM, SCALAR>>(
            new ConstraintsContainerDms<STATE_DIM, CONTROL_DIM, SCALAR>(
                optVariablesDms_, timeGrid_, shotContainers_, discretizedConstraints_, x0, settings_));

        this->optVariables_->resizeConstraintVars(this->getConstraintsCount());
    }


    /**
	 * @brief      Destructor
	 */
    ~DmsProblem() override = default;

    void updateProblem() override
    {
        controlSpliner_->computeSpline(optVariablesDms_->getOptimizedInputs().toImplementation());
        for (auto shotContainer : shotContainers_)
            shotContainer->reset();
    }

    /**
	 * @brief      Updates the settings
	 *
	 * @param[in]  settings  New dms settings
	 */
    void configure(const DmsSettings& settings) { settings_ = settings; }
    /**
	 * @brief      Retrieves the solution state trajectory at every shot
	 *
	 * @return     The state solution trajectory
	 */
    const state_vector_array_t& getStateSolution() { return optVariablesDms_->getOptimizedStates(); }
    /**
	 * @brief      Retrieves the solution control trajectory at every shot
	 *
	 * @return     The control solution trajectory
	 */
    const control_vector_array_t& getInputSolution() { return optVariablesDms_->getOptimizedInputs(); }
    /**
	 * @brief      Retrieves the solution time trajectory at every shot
	 *
	 * @return     The time solution trajectory
	 */
    const time_array_t& getTimeSolution() { return timeGrid_->toImplementation(); }
    /**
	 * @brief      Retrieves a dense state solution trajectory
	 *
	 * @return     The dense state solution trajectory
	 */
    const state_vector_array_t& getStateTrajectory() { return optVariablesDms_->getOptimizedStates(); }
    /**
	 * @brief      Retrieves a dense input solution trajectory
	 *
	 * @return     The dense control solution trajectory
	 */
    const control_vector_array_t& getInputTrajectory() { return optVariablesDms_->getOptimizedInputs(); }
    /**
	 * @brief      Retrieves a dense time solution trajectory
	 *
	 * @return     The dense time solution trajectory
	 */
    const time_array_t& getTimeArray() { return timeGrid_->toImplementation(); }
    /**
	 * @brief      Sets the initial guess of the optimization
	 *
	 * @param[in]  x_init_guess  The state trajectory initial guess
	 * @param[in]  u_init_guess  The control trajectory initial guess
	 * @param[in]  t_init_guess  The time trajectory initial guess
	 */
    void setInitialGuess(const state_vector_array_t& x_init_guess,
        const control_vector_array_t& u_init_guess,
        const time_array_t& t_init_guess = time_array_t(0.0))
    {
        optVariablesDms_->setInitGuess(x_init_guess, u_init_guess);
    }

    /**
	 * @brief      Return the timehorizon of the problem
	 *
	 * @return     The time horizon
	 */
    const core::Time getTimeHorizon() const { return timeGrid_->getTimeHorizon(); }
    /**
	 * @brief      Updates the timehorizon
	 *
	 * @param[in]  tf    The new time horizon
	 */
    void changeTimeHorizon(const SCALAR tf) { timeGrid_->changeTimeHorizon(tf); }
    /**
	 * @brief      Updates the initial state
	 *
	 * @param[in]  x0    The new inital state
	 */
    void changeInitialState(const state_vector_t& x0)
    {
        // constraints_->changeInitialConstraint(x0);
        optVariablesDms_->changeInitialState(x0);
    }

    /**
	 * @brief      Prints the solution trajectories
	 */
    void printSolution() { optVariablesDms_->printoutSolution(); }

private:
    DmsSettings settings_;

    std::shared_ptr<ConstraintDiscretizer<STATE_DIM, CONTROL_DIM, SCALAR>> discretizedConstraints_;

    std::vector<std::shared_ptr<ShotContainer<STATE_DIM, CONTROL_DIM, SCALAR>>> shotContainers_;
    std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>> optVariablesDms_;
    std::shared_ptr<SplinerBase<control_vector_t, SCALAR>> controlSpliner_;
    std::shared_ptr<tpl::TimeGrid<SCALAR>> timeGrid_;

    state_vector_array_t stateSolutionDense_;
    control_vector_array_t inputSolutionDense_;
    time_array_t timeSolutionDense_;
};


}  // namespace optcon
}  // namespace ct
