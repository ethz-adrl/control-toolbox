/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/optcon/problem/ContinuousOptConProblem.h>
#include <ct/optcon/solver/OptConSolver.h>

#include <ct/optcon/dms/dms_core/DmsProblem.h>
#include <ct/optcon/dms/dms_core/DmsSettings.h>

#include <ct/optcon/nlp/Nlp>

#include <memory>

namespace ct {
namespace optcon {

/** @defgroup   DMS DMS
 *
 * @brief      The direct multiple shooting module
 */


/**
 * @ingroup    DMS
 *
 * @brief      The DMS policy used as a solution container
 *
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
struct DmsPolicy
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef DmsDimensions<STATE_DIM, CONTROL_DIM, SCALAR> DIMENSIONS;
    typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
    typedef typename DIMENSIONS::control_vector_array_t control_vector_array_t;
    typedef typename DIMENSIONS::time_array_t time_array_t;

    state_vector_array_t xSolution_;
    control_vector_array_t uSolution_;
    time_array_t tSolution_;
};


/**
 * @ingroup    DMS
 *
 * @brief      Class to solve a specfic DMS problem
 *
 * An example employing different DMS solvers is given in unit test \ref oscDMSTest.cpp
 *
 * @tparam     STATE_DIM    The state dimension
 * @tparam     CONTROL_DIM  The control dimension
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class DmsSolver : public OptConSolver<DmsSolver<STATE_DIM, CONTROL_DIM, SCALAR>,
                      DmsPolicy<STATE_DIM, CONTROL_DIM, SCALAR>,
                      DmsSettings,
                      STATE_DIM,
                      CONTROL_DIM,
                      SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef OptConSolver<DmsSolver<STATE_DIM, CONTROL_DIM, SCALAR>,
        DmsPolicy<STATE_DIM, CONTROL_DIM, SCALAR>,
        DmsSettings,
        STATE_DIM,
        CONTROL_DIM>
        Base;

    typedef DmsDimensions<STATE_DIM, CONTROL_DIM, SCALAR> DIMENSIONS;
    typedef typename DIMENSIONS::state_vector_t state_vector_t;
    typedef typename DIMENSIONS::control_vector_array_t control_vector_array_t;
    typedef typename DIMENSIONS::control_vector_t control_vector_t;
    typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
    typedef typename DIMENSIONS::time_array_t time_array_t;

    typedef DmsPolicy<STATE_DIM, CONTROL_DIM, SCALAR> Policy_t;

    typedef ContinuousOptConProblem<STATE_DIM, CONTROL_DIM, SCALAR> OptConProblem_t;

    /**
	 * @brief      Custom constructor, converts the optcon problem to a DMS problem
	 *
	 * @param[in]  problem      The optimal control problem
	 * @param[in]  settingsDms  The dms settings
	 */
    DmsSolver(const ContinuousOptConProblem<STATE_DIM, CONTROL_DIM, SCALAR> problem, DmsSettings settingsDms)
        : nlpSolver_(nullptr), settings_(settingsDms)
    {
        // Create system, linearsystem and costfunction instances
        this->setProblem(problem);

        dmsProblem_ =
            std::shared_ptr<DmsProblem<STATE_DIM, CONTROL_DIM, SCALAR>>(new DmsProblem<STATE_DIM, CONTROL_DIM, SCALAR>(
                settingsDms, this->systems_, this->linearSystems_, this->costFunctions_, this->inputBoxConstraints_,
                this->stateBoxConstraints_, this->generalConstraints_, x0_));

        // SNOPT only works for the double type
        if (settingsDms.solverSettings_.solverType_ == NlpSolverType::SNOPT)
            nlpSolver_ = std::shared_ptr<SnoptSolver>(new SnoptSolver(dmsProblem_, settingsDms.solverSettings_));
        else if (settingsDms.solverSettings_.solverType_ == NlpSolverType::IPOPT)
            nlpSolver_ = std::shared_ptr<IpoptSolver>(new IpoptSolver(dmsProblem_, settingsDms.solverSettings_));
        else
            std::cout << "Unknown solver type... Exiting" << std::endl;

        configure(settingsDms);
    }


    /**
	 * @brief      Destructor
	 */
    ~DmsSolver() override = default;

    void configure(const DmsSettings& settings) override
    {
        dmsProblem_->configure(settings);
        dmsProblem_->changeTimeHorizon(tf_);
        dmsProblem_->changeInitialState(x0_);
    }

    bool solve() override
    {
        if (!nlpSolver_->isInitialized())
            nlpSolver_->configure(settings_.solverSettings_);

        return nlpSolver_->solve();
    }

    const Policy_t& getSolution() override
    {
        policy_.xSolution_ = dmsProblem_->getStateSolution();
        policy_.uSolution_ = dmsProblem_->getInputSolution();
        policy_.tSolution_ = dmsProblem_->getTimeSolution();
        return policy_;
    }

    const core::StateTrajectory<STATE_DIM, SCALAR> getStateTrajectory() const override
    {
        return core::StateTrajectory<STATE_DIM, SCALAR>(dmsProblem_->getTimeArray(), dmsProblem_->getStateTrajectory());
    }

    const core::ControlTrajectory<CONTROL_DIM, SCALAR> getControlTrajectory() const override
    {
        return core::ControlTrajectory<CONTROL_DIM, SCALAR>(
            dmsProblem_->getTimeArray(), dmsProblem_->getInputTrajectory());
    }

    const core::tpl::TimeArray<SCALAR>& getTimeArray() const override { return dmsProblem_->getTimeArray(); }
    void setInitialGuess(const Policy_t& initialGuess) override
    {
        dmsProblem_->setInitialGuess(initialGuess.xSolution_, initialGuess.uSolution_);
    }

    SCALAR getTimeHorizon() const override { return dmsProblem_->getTimeHorizon(); }
    void changeTimeHorizon(const SCALAR& tf) override
    {
        tf_ = tf;
        if (dmsProblem_)
            dmsProblem_->changeTimeHorizon(tf);
    }

    void changeInitialState(const core::StateVector<STATE_DIM, SCALAR>& x0) override
    {
        x0_ = x0;
        if (dmsProblem_)
            dmsProblem_->changeInitialState(x0);
    }

    void changeCostFunction(const typename Base::OptConProblem_t::CostFunctionPtr_t& cf) override
    {
        this->getCostFunctionInstances().resize(settings_.N_);
        if (cf)
            for (size_t i = 0; i < settings_.N_; i++)
                this->getCostFunctionInstances()[i] = typename Base::OptConProblem_t::CostFunctionPtr_t(cf->clone());
    }

    void changeNonlinearSystem(const typename Base::OptConProblem_t::DynamicsPtr_t& dyn) override
    {
        this->getNonlinearSystemsInstances().resize(settings_.N_);

        if (dyn)
            for (size_t i = 0; i < settings_.N_; i++)
                this->getNonlinearSystemsInstances()[i] = typename Base::OptConProblem_t::DynamicsPtr_t(dyn->clone());
    }

    void changeLinearSystem(const typename Base::OptConProblem_t::LinearPtr_t& lin) override
    {
        this->getLinearSystemsInstances().resize(settings_.N_);

        if (lin)
            for (size_t i = 0; i < settings_.N_; i++)
                this->getLinearSystemsInstances()[i] = typename Base::OptConProblem_t::LinearPtr_t(lin->clone());
    }

    void changeInputBoxConstraints(const typename Base::OptConProblem_t::ConstraintPtr_t con) override
    {
        this->getInputBoxConstraintsInstances().push_back(
            typename Base::OptConProblem_t::ConstraintPtr_t(con->clone()));
    }

    void changeStateBoxConstraints(const typename Base::OptConProblem_t::ConstraintPtr_t con) override
    {
        this->getStateBoxConstraintsInstances().push_back(
            typename Base::OptConProblem_t::ConstraintPtr_t(con->clone()));
    }

    void changeGeneralConstraints(const typename Base::OptConProblem_t::ConstraintPtr_t con) override
    {
        this->getGeneralConstraintsInstances().push_back(typename Base::OptConProblem_t::ConstraintPtr_t(con->clone()));
    }

    /**
	 * @brief      Prints out the solution trajectories of the DMS problem
	 */
    void printSolution() { dmsProblem_->printSolution(); }
    std::vector<typename OptConProblem_t::DynamicsPtr_t>& getNonlinearSystemsInstances() override { return systems_; }
    const std::vector<typename OptConProblem_t::DynamicsPtr_t>& getNonlinearSystemsInstances() const override
    {
        return systems_;
    }

    std::vector<typename OptConProblem_t::LinearPtr_t>& getLinearSystemsInstances() override { return linearSystems_; }
    const std::vector<typename OptConProblem_t::LinearPtr_t>& getLinearSystemsInstances() const override
    {
        return linearSystems_;
    }

    std::vector<typename OptConProblem_t::CostFunctionPtr_t>& getCostFunctionInstances() override
    {
        return costFunctions_;
    }
    const std::vector<typename OptConProblem_t::CostFunctionPtr_t>& getCostFunctionInstances() const override
    {
        return costFunctions_;
    }

    std::vector<typename OptConProblem_t::ConstraintPtr_t>& getInputBoxConstraintsInstances() override
    {
        return inputBoxConstraints_;
    }

    const std::vector<typename OptConProblem_t::ConstraintPtr_t>& getInputBoxConstraintsInstances() const override
    {
        return inputBoxConstraints_;
    }

    std::vector<typename OptConProblem_t::ConstraintPtr_t>& getStateBoxConstraintsInstances() override
    {
        return stateBoxConstraints_;
    }

    const std::vector<typename OptConProblem_t::ConstraintPtr_t>& getStateBoxConstraintsInstances() const override
    {
        return stateBoxConstraints_;
    }

    std::vector<typename OptConProblem_t::ConstraintPtr_t>& getGeneralConstraintsInstances() override
    {
        return generalConstraints_;
    }
    const std::vector<typename OptConProblem_t::ConstraintPtr_t>& getGeneralConstraintsInstances() const override
    {
        return generalConstraints_;
    }


private:
    std::shared_ptr<DmsProblem<STATE_DIM, CONTROL_DIM, SCALAR>> dmsProblem_; /*!<The dms problem*/
    std::shared_ptr<tpl::NlpSolver<SCALAR>> nlpSolver_; /*!<The nlp solver for solving the dmsproblem*/
    DmsSettings settings_;                              /*!<The dms settings*/

    Policy_t policy_; /*!<The solution container*/

    state_vector_t x0_; /*!<The initial state for the optimization*/
    SCALAR tf_;         /*!<The timehorizon of the problem*/

    std::vector<typename OptConProblem_t::DynamicsPtr_t> systems_;
    std::vector<typename OptConProblem_t::LinearPtr_t> linearSystems_;
    std::vector<typename OptConProblem_t::CostFunctionPtr_t> costFunctions_;
    std::vector<typename OptConProblem_t::ConstraintPtr_t> inputBoxConstraints_;
    std::vector<typename OptConProblem_t::ConstraintPtr_t> stateBoxConstraints_;
    std::vector<typename OptConProblem_t::ConstraintPtr_t> generalConstraints_;
};


}  // namespace optcon
}  // namespace ct
