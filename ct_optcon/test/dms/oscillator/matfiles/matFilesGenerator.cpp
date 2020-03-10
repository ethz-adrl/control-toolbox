/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/optcon/optcon.h>

#include "oscDMSTest_settings.h"

#ifdef MATLAB
#include <ct/optcon/matlab.hpp>
#endif


namespace ct {
namespace optcon {
namespace example {

using namespace ct;
using namespace optcon;


class MatFilesGenerator
{
public:
    typedef DmsDimensions<2, 1> OscDimensions;

    MatFilesGenerator() : w_n_(0.5), zeta_(0.01)
    {
        matlabPathIPOPT_ = std::string(DATA_DIR) + "/solutionIpopt.mat";
        matlabPathSNOPT_ = std::string(DATA_DIR) + "/solutionSnopt.mat";
        settings_.N_ = 25;
        settings_.T_ = 5.0;
        settings_.nThreads_ = 2;
        settings_.splineType_ = DmsSettings::PIECEWISE_LINEAR;
        settings_.costEvaluationType_ = DmsSettings::FULL;
        settings_.objectiveType_ = DmsSettings::KEEP_TIME_AND_GRID;
        settings_.h_min_ = 0.1;  // minimum admissible distance between two nodes in [sec]
        settings_.integrationType_ = DmsSettings::RK4;
        settings_.dt_sim_ = 0.01;
        settings_.absErrTol_ = 1e-8;
        settings_.relErrTol_ = 1e-8;
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

    void generateMatFilesIPOPT()
    {
        settings_.solverSettings_.solverType_ = NlpSolverType::IPOPT;

        generalConstraints_ = std::shared_ptr<ct::optcon::ConstraintContainerAnalytical<2, 1>>(
            new ct::optcon::ConstraintContainerAnalytical<2, 1>());

        std::shared_ptr<TerminalConstraint<2, 1>> termConstraint(new TerminalConstraint<2, 1>(x_final_));

        termConstraint->setName("crazyTerminalConstraint");
        generalConstraints_->addTerminalConstraint(termConstraint, true);

        ContinuousOptConProblem<2, 1> optProblem(oscillator_, costFunction_);
        optProblem.setInitialState(x_0_);

        optProblem.setTimeHorizon(settings_.T_);
        optProblem.setGeneralConstraints(generalConstraints_);

        dmsPlanner_ = std::shared_ptr<DmsSolver<2, 1>>(new DmsSolver<2, 1>(optProblem, settings_));

        calcInitGuess();
        dmsPlanner_->setInitialGuess(initialPolicy_);
        dmsPlanner_->solve();
        solutionPolicy_ = dmsPlanner_->getSolution();

        //Solution Containers
        OscDimensions::state_vector_array_t stateSolutionIpopt;
        OscDimensions::control_vector_array_t inputSolutionIpopt;
        OscDimensions::time_array_t timeSolutionIpopt;
        stateSolutionIpopt = solutionPolicy_.xSolution_;
        inputSolutionIpopt = solutionPolicy_.uSolution_;
        timeSolutionIpopt = solutionPolicy_.tSolution_;


#ifdef MATLAB
        matlab::MatFile matFile;
        matFile.open(matlabPathIPOPT_);
        matFile.put("stateDmsIpopt", stateSolutionIpopt.toImplementation());
        matFile.put("inputDmsIpopt", inputSolutionIpopt.toImplementation());
        matFile.put("timeDmsIpopt", timeSolutionIpopt.toEigenTrajectory());
        matFile.close();
#endif  //MATLAB
    }

    void generateMatFilesSNOPT()
    {
        settings_.solverSettings_.solverType_ = NlpSolverType::SNOPT;

        generalConstraints_ = std::shared_ptr<ct::optcon::ConstraintContainerAnalytical<2, 1>>(
            new ct::optcon::ConstraintContainerAnalytical<2, 1>());

        std::shared_ptr<TerminalConstraint<2, 1>> termConstraint(new TerminalConstraint<2, 1>(x_final_));

        termConstraint->setName("crazyTerminalConstraint");
        generalConstraints_->addTerminalConstraint(termConstraint, true);

        ContinuousOptConProblem<2, 1> optProblem(oscillator_, costFunction_);
        optProblem.setInitialState(x_0_);

        optProblem.setTimeHorizon(settings_.T_);
        optProblem.setGeneralConstraints(generalConstraints_);
        generalConstraints_->initialize();

        dmsPlanner_ = std::shared_ptr<DmsSolver<2, 1>>(new DmsSolver<2, 1>(optProblem, settings_));

        calcInitGuess();
        dmsPlanner_->setInitialGuess(initialPolicy_);
        dmsPlanner_->solve();
        solutionPolicy_ = dmsPlanner_->getSolution();

        //Solution Containers
        OscDimensions::state_vector_array_t stateSolutionSnopt;
        OscDimensions::control_vector_array_t inputSolutionSnopt;
        OscDimensions::time_array_t timeSolutionSnopt;
        stateSolutionSnopt = solutionPolicy_.xSolution_;
        inputSolutionSnopt = solutionPolicy_.uSolution_;
        timeSolutionSnopt = solutionPolicy_.tSolution_;

#ifdef MATLAB
        matlab::MatFile matFile;
        matFile.open(matlabPathSNOPT_);
        matFile.put("stateDmsSnopt", stateSolutionSnopt.toImplementation());
        matFile.put("inputDmsSnopt", inputSolutionSnopt.toImplementation());
        matFile.put("timeDmsSnopt", timeSolutionSnopt.toEigenTrajectory());
        matFile.close();
#endif  //MATLAB
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


    DmsPolicy<2, 1> initialPolicy_;
    DmsPolicy<2, 1> solutionPolicy_;
    OscDimensions::state_vector_array_t x_initguess_;
    OscDimensions::control_vector_array_t u_initguess_;

    OscDimensions::state_vector_t x_0_;
    OscDimensions::state_vector_t x_final_;
    OscDimensions::state_matrix_t Q_;
    OscDimensions::state_matrix_t Q_final_;
    OscDimensions::control_matrix_t R_;
    OscDimensions::control_vector_t u_des_;
};

}  // namespace example
}  // namespace optcon
}  // namespace ct


/*!
 * This executable is used to generate the reference matfiles for the unit tests.
 */
int main(int argc, char* argv[])
{
    using namespace ct::optcon::example;

    MatFilesGenerator oscDms;
    oscDms.initialize();
#ifdef BUILD_WITH_SNOPT_SUPPORT
    std::cout << "Generating Mat Files using SNOPT:" << std::endl;
    oscDms.generateMatFilesSNOPT();
#endif  // BUILD_WITH_IPOPT_SUPPORT

#ifdef BUILD_WITH_IPOPT_SUPPORT
    std::cout << "Generating Mat Files using IPOPT:" << std::endl;
    oscDms.generateMatFilesIPOPT();
#endif  // BUILD_WITH_IPOPT_SUPPORT

    return 0;
}
