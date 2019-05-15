/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

/*!
 * This file implements dms unit tests.
 * For more intuitive examples, visit the tutorial.
 * \example oscDMSTest.cpp
 */

#include <ct/optcon/optcon.h>
#include <gtest/gtest.h>

#include "oscDMSTest_settings.h"


#ifdef MATLAB
#include <ct/optcon/matlab.hpp>
#endif

namespace ct {
namespace optcon {
namespace example {

class OscDms
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef DmsDimensions<2, 1> OscDimensions;

    OscDms() : w_n_(0.5), zeta_(0.01)
    {
        matlabPathIPOPT_ = std::string(DATA_DIR) + "/solutionIpopt.mat";
        matlabPathSNOPT_ = std::string(DATA_DIR) + "/solutionSnopt.mat";

        settings_.N_ = 25;
        settings_.T_ = 5.0;
        settings_.nThreads_ = 4;
        settings_.splineType_ = DmsSettings::PIECEWISE_LINEAR;
        settings_.costEvaluationType_ = DmsSettings::FULL;
        settings_.objectiveType_ = DmsSettings::KEEP_TIME_AND_GRID;
        settings_.h_min_ = 0.1;  // minimum admissible distance between two nodes in [sec]
        settings_.integrationType_ = DmsSettings::RK4;
        settings_.dt_sim_ = 0.01;
        settings_.absErrTol_ = 1e-8;
        settings_.relErrTol_ = 1e-8;

        settings_.print();
    }

    ~OscDms() { std::cout << "Oscillator dms destructor called" << std::endl; }
    void getIpoptMatlabTrajectories()
    {
#ifdef MATLAB
        matFileIpopt_.open(matlabPathIPOPT_, matlab::MatFile::READ);
        assert(matFileIpopt_.isOpen());
        matFileIpopt_.get("stateDmsIpopt", stateMatIpopt_.toImplementation());
        matFileIpopt_.get("inputDmsIpopt", inputMatIpopt_.toImplementation());
        std::vector<Eigen::Matrix<double, 1, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 1, 1>>> timeEigen;
        matFileIpopt_.get("timeDmsIpopt", timeEigen);
        timeMatIpopt_.fromEigenTrajectory(timeEigen);
        matFileIpopt_.close();
#endif
    }

    void getSnoptMatlabTrajectories()
    {
#ifdef MATLAB
        matFileSnopt_.open(matlabPathSNOPT_, matlab::MatFile::READ);
        assert(matFileSnopt_.isOpen());
        matFileSnopt_.get("stateDmsSnopt", stateMatSnopt_.toImplementation());
        matFileSnopt_.get("inputDmsSnopt", inputMatSnopt_.toImplementation());
        std::vector<Eigen::Matrix<double, 1, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 1, 1>>> timeEigen;
        matFileSnopt_.get("timeDmsSnopt", timeEigen);
        timeMatSnopt_.fromEigenTrajectory(timeEigen);
        matFileSnopt_.close();
#endif
    }

    void initialize()
    {
        oscillator_ = std::shared_ptr<ct::core::SecondOrderSystem>(new ct::core::SecondOrderSystem(w_n_, zeta_));
        x_0_ << 0.0, 0.0;
        x_final_ << 2.0, -1.0;
        Q_ << 0.0, 0.0, 0.0, 10.0;

        Q_final_ << 0.0, 0.0, 0.0, 0.0;

        R_ << 0.001;
        u_des_ << 0.0;

        costFunction_ = std::shared_ptr<ct::optcon::CostFunctionQuadratic<2, 1>>(
            new ct::optcon::CostFunctionQuadraticSimple<2, 1>(Q_, R_, x_final_, u_des_, x_final_, Q_final_));
    }

    void getIpoptSolution()
    {
        settings_.solverSettings_.solverType_ = NlpSolverType::IPOPT;

        generalConstraints_ = std::shared_ptr<ct::optcon::ConstraintContainerAnalytical<2, 1>>(
            new ct::optcon::ConstraintContainerAnalytical<2, 1>());

        std::shared_ptr<TerminalConstraint<2, 1>> termConstraint(new TerminalConstraint<2, 1>(x_final_));

        termConstraint->setName("TerminalConstraint");
        generalConstraints_->addTerminalConstraint(termConstraint, true);
        generalConstraints_->initialize();

        ContinuousOptConProblem<2, 1> optProblem(oscillator_, costFunction_);
        optProblem.setInitialState(x_0_);

        optProblem.setTimeHorizon(settings_.T_);
        optProblem.setGeneralConstraints(generalConstraints_);
        dmsPlanner_ = std::shared_ptr<DmsSolver<2, 1>>(new DmsSolver<2, 1>(optProblem, settings_));

        calcInitGuess();
        dmsPlanner_->setInitialGuess(initialPolicy_);
        dmsPlanner_->solve();
        solutionPolicy_ = dmsPlanner_->getSolution();


        stateSolutionIpopt_ = solutionPolicy_.xSolution_;
        inputSolutionIpopt_ = solutionPolicy_.uSolution_;
        timeSolutionIpopt_ = solutionPolicy_.tSolution_;
    }

    void getSnoptSolution()
    {
        settings_.solverSettings_.solverType_ = NlpSolverType::SNOPT;

        generalConstraints_ = std::shared_ptr<ct::optcon::ConstraintContainerAnalytical<2, 1>>(
            new ct::optcon::ConstraintContainerAnalytical<2, 1>());

        std::shared_ptr<TerminalConstraint<2, 1>> termConstraint(new TerminalConstraint<2, 1>(x_final_));

        termConstraint->setName("TerminalConstraint");
        generalConstraints_->addTerminalConstraint(termConstraint, true);
        generalConstraints_->initialize();

        ContinuousOptConProblem<2, 1> optProblem(oscillator_, costFunction_);
        optProblem.setInitialState(x_0_);

        optProblem.setTimeHorizon(settings_.T_);
        optProblem.setGeneralConstraints(generalConstraints_);
        dmsPlanner_ = std::shared_ptr<DmsSolver<2, 1>>(new DmsSolver<2, 1>(optProblem, settings_));

        calcInitGuess();
        dmsPlanner_->setInitialGuess(initialPolicy_);
        dmsPlanner_->solve();
        solutionPolicy_ = dmsPlanner_->getSolution();

        stateSolutionSnopt_ = solutionPolicy_.xSolution_;
        inputSolutionSnopt_ = solutionPolicy_.uSolution_;
        timeSolutionSnopt_ = solutionPolicy_.tSolution_;
    }

    void compareIpoptSolutions()
    {
#ifdef MATLAB
        ASSERT_TRUE(stateSolutionIpopt_.size() == stateMatIpopt_.size());
        for (size_t i = 0; i < stateSolutionIpopt_.size(); ++i)
        {
            ASSERT_TRUE(stateSolutionIpopt_[i].isApprox(stateMatIpopt_[i]));
            ASSERT_TRUE(inputSolutionIpopt_[i].isApprox(inputMatIpopt_[i]));
            ASSERT_TRUE(timeSolutionIpopt_[i] == timeMatIpopt_[i]);
        }
#endif
    }

    void compareSnoptSolutions()
    {
#ifdef MATLAB
        ASSERT_TRUE(stateSolutionSnopt_.size() == stateMatSnopt_.size());
        for (size_t i = 0; i < stateSolutionSnopt_.size(); ++i)
        {
            ASSERT_TRUE(stateSolutionSnopt_[i].isApprox(stateMatSnopt_[i]));
            ASSERT_TRUE(inputSolutionSnopt_[i].isApprox(inputMatSnopt_[i]));
            ASSERT_TRUE(timeSolutionSnopt_[i] == timeMatSnopt_[i]);
        }
#endif
    }


private:
    void calcInitGuess()
    {
        x_initguess_.resize(settings_.N_ + 1, OscDimensions::state_vector_t::Zero());
        u_initguess_.resize(settings_.N_ + 1, OscDimensions::control_vector_t::Zero());
        for (size_t i = 0; i < settings_.N_ + 1; ++i)
        {
            x_initguess_[i] = x_0_ + (x_final_ - x_0_) * (i / settings_.N_);
        }

        initialPolicy_.xSolution_ = x_initguess_;
        initialPolicy_.uSolution_ = u_initguess_;
    }

    double w_n_;
    double zeta_;
    std::shared_ptr<ct::core::SecondOrderSystem> oscillator_;

    std::string matlabPathIPOPT_;
    std::string matlabPathSNOPT_;

    DmsSettings settings_;
    std::shared_ptr<DmsSolver<2, 1>> dmsPlanner_;
    std::shared_ptr<ct::optcon::CostFunctionQuadratic<2, 1>> costFunction_;
    std::shared_ptr<ct::optcon::ConstraintContainerAnalytical<2, 1>> generalConstraints_;

    OscDimensions::state_vector_t x_0_;
    OscDimensions::state_vector_t x_final_;
    OscDimensions::state_matrix_t Q_;
    OscDimensions::state_matrix_t Q_final_;
    OscDimensions::control_matrix_t R_;
    OscDimensions::control_vector_t u_des_;

    DmsPolicy<2, 1> initialPolicy_;
    DmsPolicy<2, 1> solutionPolicy_;
    OscDimensions::state_vector_array_t x_initguess_;
    OscDimensions::control_vector_array_t u_initguess_;

    //Solutions loaded from Mat Files
    OscDimensions::state_vector_array_t stateMatIpopt_;
    OscDimensions::control_vector_array_t inputMatIpopt_;
    OscDimensions::time_array_t timeMatIpopt_;
    OscDimensions::state_vector_array_t stateMatSnopt_;
    OscDimensions::control_vector_array_t inputMatSnopt_;
    OscDimensions::time_array_t timeMatSnopt_;

    //Solution obtained by IPOPT, SNOPT
    OscDimensions::state_vector_array_t stateSolutionIpopt_;
    OscDimensions::control_vector_array_t inputSolutionIpopt_;
    OscDimensions::time_array_t timeSolutionIpopt_;
    OscDimensions::state_vector_array_t stateSolutionSnopt_;
    OscDimensions::control_vector_array_t inputSolutionSnopt_;
    OscDimensions::time_array_t timeSolutionSnopt_;

#ifdef MATLAB
    matlab::MatFile matFileIpopt_;
    matlab::MatFile matFileSnopt_;
#endif  //MATLAB
};

TEST(DmsTest, OscDmsTest)
{
    OscDms oscDms;
    oscDms.initialize();
#ifdef BUILD_WITH_SNOPT_SUPPORT
    oscDms.getSnoptMatlabTrajectories();
    oscDms.getSnoptSolution();
    oscDms.compareSnoptSolutions();
#endif  // BUILD_WITH_SNOPT_SUPPORT

#ifdef BUILD_WITH_IPOPT_SUPPORT
    oscDms.getIpoptMatlabTrajectories();
    oscDms.getIpoptSolution();
    oscDms.compareIpoptSolutions();
#endif  // BUILD_WITH_IPOPT_SUPPORT
}


}  // namespace example
}  // namespace optcon
}  // namespace ct

/*!
 * This unit test applies Direct Multiple Shooting to an oscillator system, uses different solvers and compares the outputs.
 * \example oscDMSTest.cpp
 */
int main(int argc, char** argv)
{
    // using namespace ct::optcon::example;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
