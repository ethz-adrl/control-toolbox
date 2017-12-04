/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/


#pragma once


#include <ct/optcon/problem/OptConProblem.h>
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

    typedef OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR> OptConProblem_t;


    /**
	 * @brief      Custom constructor, sets up the objects needed for the Dms
	 *             algorithm depending on the settings
	 *
	 * @param[in]  settings                 The Dms settings
	 * @param[in]  systemPtrs               The non linear systems
	 * @param[in]  linearPtrs               The linearized systems
	 * @param[in]  costPtrs                 The cost function
	 * @param[in]  boxConstraints           The box constraints
	 * @param[in]  generaConstraints        The general constraints
	 * @param[in]  x0                       The initial state
	 */
    DmsProblem(DmsSettings settings,
        std::vector<typename OptConProblem_t::DynamicsPtr_t> systemPtrs,
        std::vector<typename OptConProblem_t::LinearPtr_t> linearPtrs,
        std::vector<typename OptConProblem_t::CostFunctionPtr_t> costPtrs,
        std::vector<typename OptConProblem_t::ConstraintPtr_t> boxConstraints,
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
            throw std::runtime_error("Currently we do not support adaptive time gridding");
            // wLength += settings_.N_;
        }

        this->optVariables_ = std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>>(
            new OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>(wLength, settings));

        optVariablesDms_ = std::static_pointer_cast<OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>>(this->optVariables_);

        if (boxConstraints.size() > 0 || generalConstraints.size() > 0)
            discretizedConstraints_ = std::shared_ptr<ConstraintDiscretizer<STATE_DIM, CONTROL_DIM, SCALAR>>(
                new ConstraintDiscretizer<STATE_DIM, CONTROL_DIM, SCALAR>(
                    optVariablesDms_, controlSpliner_, timeGrid_, settings_.N_));

        if (boxConstraints.size() > 0)
            discretizedConstraints_->setBoxConstraints(boxConstraints.front());

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

    void generateAndCompileCode(
        std::vector<std::shared_ptr<core::ControlledSystem<STATE_DIM, CONTROL_DIM, ct::core::ADCGScalar>>> systemPtrs,
        std::vector<std::shared_ptr<core::LinearSystem<STATE_DIM, CONTROL_DIM, ct::core::ADCGScalar>>> linearPtrs,
        std::vector<std::shared_ptr<optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, ct::core::ADCGScalar>>>
            costPtrs,
        std::vector<std::shared_ptr<optcon::LinearConstraintContainer<STATE_DIM, CONTROL_DIM, ct::core::ADCGScalar>>>
            boxConstraints,
        std::vector<std::shared_ptr<optcon::LinearConstraintContainer<STATE_DIM, CONTROL_DIM, ct::core::ADCGScalar>>>
            generalConstraints,
        const ct::core::StateVector<STATE_DIM, ct::core::ADCGScalar>& x0)
    {
        typedef ct::core::ADCGScalar ScalarCG;
        size_t wLength = (settings_.N_ + 1) * (STATE_DIM + CONTROL_DIM);

        std::shared_ptr<tpl::TimeGrid<ScalarCG>> timeGrid(
            new tpl::TimeGrid<ScalarCG>(settings_.N_, ScalarCG(settings_.T_)));

        std::shared_ptr<SplinerBase<ct::core::ControlVector<CONTROL_DIM, ScalarCG>, ScalarCG>> controlSpliner;

        switch (settings_.splineType_)
        {
            case DmsSettings::ZERO_ORDER_HOLD:
            {
                controlSpliner =
                    std::shared_ptr<ZeroOrderHoldSpliner<ct::core::ControlVector<CONTROL_DIM, ScalarCG>, ScalarCG>>(
                        new ZeroOrderHoldSpliner<ct::core::ControlVector<CONTROL_DIM, ScalarCG>, ScalarCG>(timeGrid));
                break;
            }
            case DmsSettings::PIECEWISE_LINEAR:
            {
                controlSpliner =
                    std::shared_ptr<LinearSpliner<ct::core::ControlVector<CONTROL_DIM, ScalarCG>, ScalarCG>>(
                        new LinearSpliner<ct::core::ControlVector<CONTROL_DIM, ScalarCG>, ScalarCG>(timeGrid));
                break;
            }
            default:
                throw(std::runtime_error("Unknown spline type"));
        }

        std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM, ScalarCG>> optVariablesDms(
            new OptVectorDms<STATE_DIM, CONTROL_DIM, ScalarCG>(wLength, settings_));


        std::shared_ptr<ConstraintDiscretizer<STATE_DIM, CONTROL_DIM, ScalarCG>> discretizedConstraints;
        if (boxConstraints.size() > 0 || generalConstraints.size() > 0)
            discretizedConstraints = std::shared_ptr<ConstraintDiscretizer<STATE_DIM, CONTROL_DIM, ScalarCG>>(
                new ConstraintDiscretizer<STATE_DIM, CONTROL_DIM, ScalarCG>(
                    optVariablesDms, controlSpliner, timeGrid, settings_.N_));

        if (boxConstraints.size() > 0)
            discretizedConstraints->setBoxConstraints(boxConstraints.front());

        if (generalConstraints.size() > 0)
            discretizedConstraints->setGeneralConstraints(generalConstraints.front());

        std::vector<std::shared_ptr<ShotContainer<STATE_DIM, CONTROL_DIM, ScalarCG>>> shotContainers;
        for (size_t shotIdx = 0; shotIdx < settings_.N_; shotIdx++)
        {
            std::shared_ptr<ControllerDms<STATE_DIM, CONTROL_DIM, ScalarCG>> newController(
                new ControllerDms<STATE_DIM, CONTROL_DIM, ScalarCG>(controlSpliner, shotIdx));
            systemPtrs[shotIdx]->setController(newController);
            linearPtrs[shotIdx]->setController(newController);

            size_t nIntegrationSteps =
                (timeGrid_->getShotEndTime(shotIdx) - timeGrid_->getShotStartTime(shotIdx)) / settings_.dt_sim_ + 0.5;

            shotContainers.push_back(std::shared_ptr<ShotContainer<STATE_DIM, CONTROL_DIM, ScalarCG>>(
                new ShotContainer<STATE_DIM, CONTROL_DIM, ScalarCG>(systemPtrs[shotIdx], linearPtrs[shotIdx],
                    costPtrs[shotIdx], optVariablesDms, controlSpliner, timeGrid, shotIdx, settings_,
                    nIntegrationSteps)));
        }

        std::shared_ptr<tpl::DiscreteCostEvaluatorBase<ScalarCG>> costEvaluator;

        switch (settings_.costEvaluationType_)
        {
            case DmsSettings::SIMPLE:
            {
                costEvaluator = std::shared_ptr<CostEvaluatorSimple<STATE_DIM, CONTROL_DIM, ScalarCG>>(
                    new CostEvaluatorSimple<STATE_DIM, CONTROL_DIM, ScalarCG>(
                        costPtrs.front(), optVariablesDms, timeGrid, settings_));
                break;
            }
            case DmsSettings::FULL:
            {
                costEvaluator = std::shared_ptr<CostEvaluatorFull<STATE_DIM, CONTROL_DIM, ScalarCG>>(
                    new CostEvaluatorFull<STATE_DIM, CONTROL_DIM, ScalarCG>(
                        costPtrs.front(), optVariablesDms, controlSpliner, shotContainers, settings_));
                break;
            }
            default:
                throw(std::runtime_error("Unknown cost evaluation type"));
        }

        optVariablesDms->resizeConstraintVars(this->getConstraintsCount());

        std::shared_ptr<ConstraintsContainerDms<STATE_DIM, CONTROL_DIM, ScalarCG>> constraints(
            new ConstraintsContainerDms<STATE_DIM, CONTROL_DIM, ScalarCG>(
                optVariablesDms, timeGrid, shotContainers, discretizedConstraints, x0, settings_));

        if (settings_.solverSettings_.useGeneratedCostGradient_)
        {
            std::function<Eigen::Matrix<ScalarCG, 1, 1>(const Eigen::Matrix<ScalarCG, Eigen::Dynamic, 1>&)> fCost = [&](
                const Eigen::Matrix<ScalarCG, Eigen::Dynamic, 1>& xOpt) {
                optVariablesDms->setOptimizationVars(xOpt);

                controlSpliner->computeSpline(optVariablesDms->getOptimizedInputs().toImplementation());
                for (auto shotContainer : shotContainers)
                    shotContainer->reset();

                Eigen::Matrix<ScalarCG, 1, 1> out;
                out << costEvaluator->eval();
                return out;
            };

            settings_.cppadSettings_.createJacobian_ = true;

            this->costCodegen_ = std::shared_ptr<ct::core::DerivativesCppadJIT<-1, 1>>(
                new ct::core::DerivativesCppadJIT<-1, 1>(fCost, this->getVarCount()));
            this->costCodegen_->compileJIT(settings_.cppadSettings_, "dmsCostFunction");
        }

        if (settings_.solverSettings_.useGeneratedConstraintJacobian_)
        {
            std::function<Eigen::Matrix<ScalarCG, Eigen::Dynamic, 1>(const Eigen::Matrix<ScalarCG, Eigen::Dynamic, 1>&)>
                fConstraints = [&](const Eigen::Matrix<ScalarCG, Eigen::Dynamic, 1>& xOpt) {
                    optVariablesDms->setOptimizationVars(xOpt);

                    controlSpliner->computeSpline(optVariablesDms->getOptimizedInputs().toImplementation());
                    for (auto shotContainer : shotContainers)
                        shotContainer->reset();


                    Eigen::Matrix<ScalarCG, Eigen::Dynamic, 1> out(
                        this->getConstraintsCount());  // out.resize(this->getConstraintsCount, 1);
                    constraints->evalConstraints(out);
                    return out;
                };

            settings_.cppadSettings_.createJacobian_ = false;

            this->constraintsCodegen_ =
                std::shared_ptr<ct::core::DerivativesCppadJIT<-1, -1>>(new ct::core::DerivativesCppadJIT<-1, -1>(
                    fConstraints, this->getVarCount(), this->getConstraintsCount()));
            this->constraintsCodegen_->compileJIT(settings_.cppadSettings_, "dmsConstraints");
        }
    }

    /**
	 * @brief      Destructor
	 */
    virtual ~DmsProblem() {}
    virtual void updateProblem() override
    {
        controlSpliner_->computeSpline(optVariablesDms_->getOptimizedInputs().toImplementation());
        for (auto shotContainer : shotContainers_)
            shotContainer->reset();

        // if(settings_.objectiveType_ == DmsSettings::OPTIMIZE_GRID)
        // {
        // 	// std::cout << "optVariablesDms_->getOptimizedTimeSegments(): " << optVariablesDms_->getOptimizedTimeSegments().transpose() << std::endl;
        // 	timeGrid_->updateTimeGrid(optVariablesDms_->getOptimizedTimeSegments());
        // }
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
    const state_vector_array_t& getStateTrajectory()
    {
        // stateSolutionDense_.clear();
        // for(auto shotContainer : shotContainers_)
        // 	shotContainer->integrateShot();

        // stateSolutionDense_.push_back(shotContainers_.front()->getXHistory().front());
        // for(auto shotContainer : shotContainers_)
        // {
        // 	state_vector_array_t x_traj = shotContainer->getXHistory();
        // 	for(size_t j = 1; j < x_traj.size(); ++j)
        // 		stateSolutionDense_.push_back(x_traj[j]);
        // }
        return optVariablesDms_->getOptimizedStates();
        ;
    }

    /**
	 * @brief      Retrieves a dense input solution trajectory
	 *
	 * @return     The dense control solution trajectory
	 */
    const control_vector_array_t& getInputTrajectory()
    {
        // inputSolutionDense_.clear();
        // for(auto shotContainer : shotContainers_)
        // 	shotContainer->integrateShot();

        // inputSolutionDense_.push_back(shotContainers_.front()->getUHistory().front());
        // for(auto shotContainer : shotContainers_)
        // {
        // 	control_vector_array_t u_traj = shotContainer->getUHistory();
        // 	for(size_t j = 1; j < u_traj.size(); ++j)
        // 		inputSolutionDense_.push_back(u_traj[j]);
        // }
        // return inputSolutionDense_;
        return optVariablesDms_->getOptimizedInputs();
    }

    /**
	 * @brief      Retrieves a dense time solution trajectory
	 *
	 * @return     The dense time solution trajectory
	 */
    const time_array_t& getTimeArray()
    {
        // timeSolutionDense_.clear();
        // for(auto shotContainer : shotContainers_)
        // 	shotContainer->integrateShot();

        // timeSolutionDense_.push_back(shotContainers_.front()->getTHistory().front());
        // for(auto shotContainer : shotContainers_)
        // {
        // 	time_array_t t_traj = shotContainer->getTHistory();
        // 	for(size_t j = 1; j < t_traj.size(); ++j)
        // 		timeSolutionDense_.push_back(t_traj[j]);
        // }
        // return timeSolutionDense_;
        return timeGrid_->toImplementation();
    }

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
