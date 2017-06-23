/***********************************************************************************
Copyright (c) 2016, Agile & Dexterous Robotics Lab, ETH ZURICH. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be used
      to endorse or promote products derived from this software without specific
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/

#include <cstring>
#include <iostream>
#include <memory>

#include <ct/optcon/dms/Dms>
#include <ct/optcon/costfunction/CostFunctionQuadraticSimple.hpp>

#include <gtest/gtest.h>
#include <ct/core/core.h>

#include <ct/optcon/problem/OptConProblem.h>
#include <ct/optcon/dms/dms_core/DmsSolver.h>
#include <ct/optcon/dms/dms_core/DmsSettings.h>
#include <ct/optcon/nlp/solver/NlpSolverSettings.h>

#include <ct/optcon/constraint/term/TerminalConstraint.h>
#include <ct/optcon/constraint/ConstraintContainerAnalytical.h> 

#include "oscDMSTest_settings.h"


#ifdef MATLAB
#include <ct/optcon/matlab.hpp>
#endif

namespace ct{
namespace optcon{
namespace example{

class OscDms
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	typedef DmsDimensions<2,1> OscDimensions;
	
	OscDms() :
		w_n_(0.5),
		zeta_(0.01)
	{
		matlabPathIPOPT_ = std::string(DATA_DIR) + "/solutionIpopt.mat";
		matlabPathSNOPT_ = std::string(DATA_DIR) + "/solutionSnopt.mat";

		settings_.N_ = 25;
		settings_.T_ = 5.0;
		settings_.nThreads_ = 4;
		settings_.terminalStateConstraint_ = 1;
		settings_.splineType_ = DmsSettings::PIECEWISE_LINEAR;
		settings_.costEvaluationType_ = DmsSettings::FULL;
		settings_.objectiveType_ = DmsSettings::OPTIMIZE_GRID;
		settings_.h_min_ = 0.1; // minimum admissible distance between two nodes in [sec]
		settings_.integrationType_ = DmsSettings::RK4;
		settings_.dt_sim_ = 0.01;
		settings_.integrateSens_ = 1;
		settings_.absErrTol_ = 1e-8;
		settings_.relErrTol_ = 1e-8;

		settings_.print();
	}

	~OscDms(){
		std::cout << "Oscillator dms destructor called" << std::endl;
	}

	void getIpoptMatlabTrajectories()
	{
#ifdef MATLAB
		size_t trajSize = settings_.N_ + 1;
		Eigen::MatrixXd stateTraj;
		stateTraj.resize(2, trajSize);
		Eigen::MatrixXd inputTraj;
		inputTraj.resize(1, trajSize);
		Eigen::VectorXd timeTraj;
		timeTraj.resize(trajSize);
		matFileIpopt_.open(matlabPathIPOPT_, matlab::MatFile::READ);
		assert(matFileIpopt_.isOpen());
		matFileIpopt_.get("stateDmsIpopt", stateTraj);
		matFileIpopt_.get("inputDmsIpopt", inputTraj);
		matFileIpopt_.get("timeDmsIpopt", timeTraj);
		matFileIpopt_.close();

		for(size_t i = 0; i < stateTraj.cols(); ++i)
		{
			stateMatIpopt_.push_back(stateTraj.col(i));
			inputMatIpopt_.push_back(inputTraj.col(i));
			timeMatIpopt_.push_back(timeTraj(i));
		}		
#endif
	}

	void getSnoptMatlabTrajectories()
	{
#ifdef MATLAB
		size_t trajSize = settings_.N_ + 1;
		Eigen::MatrixXd stateTraj;
		stateTraj.resize(2, trajSize);
		Eigen::MatrixXd inputTraj;
		inputTraj.resize(1, trajSize);
		Eigen::VectorXd timeTraj;
		timeTraj.resize(trajSize);

		matFileSnopt_.open(matlabPathSNOPT_, matlab::MatFile::READ);
		assert(matFileSnopt_.isOpen());
		matFileSnopt_.get("stateDmsSnopt", stateTraj);
		matFileSnopt_.get("inputDmsSnopt", inputTraj);
		matFileSnopt_.get("timeDmsSnopt", timeTraj);
		matFileSnopt_.close();

		for(size_t i = 0; i < stateTraj.cols(); ++i)
		{
			stateMatSnopt_.push_back(stateTraj.col(i));
			inputMatSnopt_.push_back(inputTraj.col(i));
			timeMatSnopt_.push_back(timeTraj(i));
		}
#endif
	}

	void initialize()
	{
		oscillator_ = std::shared_ptr<ct::core::SecondOrderSystem> (new ct::core::SecondOrderSystem(w_n_, zeta_));
		x_0_ << 0.0,0.0;
		x_final_ << 2.0, -1.0;
		Q_ << 	0.0,0.0,
				0.0,10.0;

		Q_final_ << 0.0,0.0,
					0.0,0.0;

		R_ << 0.001;
		u_des_ << 0.0;

		costFunction_ = std::shared_ptr<ct::optcon::CostFunctionQuadratic<2,1>> 
				(new ct::optcon::CostFunctionQuadraticSimple<2,1>(Q_, R_, x_final_, u_des_, x_final_, Q_final_));


	}

	void getIpoptSolution()
	{
		settings_.nlpSettings_.solverType_ = NlpSolverSettings::IPOPT;

		pureStateConstraints_ = std::shared_ptr<ct::optcon::ConstraintContainerAnalytical<2, 1>>
				(new ct::optcon::ConstraintContainerAnalytical<2, 1>());

		std::shared_ptr<TerminalConstraint<2,1>> termConstraint(new TerminalConstraint<2,1>(x_final_));

		termConstraint->setName("TerminalConstraint");
		pureStateConstraints_->addTerminalConstraint(termConstraint, true);
		pureStateConstraints_->initialize();

		OptConProblem<2,1> optProblem(oscillator_, costFunction_);
		optProblem.setInitialState(x_0_);

		optProblem.setTimeHorizon(settings_.T_);
		optProblem.setPureStateConstraints(pureStateConstraints_);
		dmsPlanner_ = std::shared_ptr<DmsSolver<2,1>> (new DmsSolver<2,1>(optProblem, settings_));

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
		settings_.nlpSettings_.solverType_ = NlpSolverSettings::SNOPT;

		pureStateConstraints_ = std::shared_ptr<ct::optcon::ConstraintContainerAnalytical<2, 1>>
				(new ct::optcon::ConstraintContainerAnalytical<2, 1>());

		std::shared_ptr<TerminalConstraint<2,1>> termConstraint(new TerminalConstraint<2,1>(x_final_));

		termConstraint->setName("TerminalConstraint");
		pureStateConstraints_->addTerminalConstraint(termConstraint, true);
		pureStateConstraints_->initialize();

		OptConProblem<2,1> optProblem(oscillator_, costFunction_);
		optProblem.setInitialState(x_0_);

		optProblem.setTimeHorizon(settings_.T_);
		optProblem.setPureStateConstraints(pureStateConstraints_);
		dmsPlanner_ = std::shared_ptr<DmsSolver<2,1>> (new DmsSolver<2,1>(optProblem, settings_));

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
		for(size_t i = 0; i < stateSolutionIpopt_.size(); ++i)
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
		for(size_t i = 0; i < stateSolutionSnopt_.size(); ++i)
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
		for(size_t i = 0; i < settings_.N_ + 1; ++i)
		{
			x_initguess_[i] = x_0_ + (x_final_ - x_0_) * (i / settings_.N_);
		}

		initialPolicy_.xSolution_ = x_initguess_;
		initialPolicy_.uSolution_ = u_initguess_;
	}

	double w_n_;
	double zeta_;
	std::shared_ptr<ct::core::SecondOrderSystem > oscillator_;

	std::string matlabPathIPOPT_;
	std::string matlabPathSNOPT_;

	DmsSettings settings_;
	std::shared_ptr<DmsSolver<2, 1>> dmsPlanner_;
	std::shared_ptr<ct::optcon::CostFunctionQuadratic<2,1> >  costFunction_;
	std::shared_ptr<ct::optcon::ConstraintContainerAnalytical<2, 1> > pureStateConstraints_;

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
	std::vector<ct::core::Time> timeMatIpopt_;
	OscDimensions::state_vector_array_t stateMatSnopt_;
	OscDimensions::control_vector_array_t inputMatSnopt_;
	std::vector<ct::core::Time> timeMatSnopt_;

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
#endif //MATLAB

};

TEST(DmsTest, OscDmsTest)
{
	OscDms oscDms;
	oscDms.initialize();
#ifdef BUILD_WITH_SNOPT_SUPPORT
	oscDms.getSnoptMatlabTrajectories();
	oscDms.getSnoptSolution();
	oscDms.compareSnoptSolutions();
#endif// BUILD_WITH_SNOPT_SUPPORT

#ifdef BUILD_WITH_IPOPT_SUPPORT
	oscDms.getIpoptMatlabTrajectories();
	oscDms.getIpoptSolution();
	oscDms.compareIpoptSolutions();
#endif // BUILD_WITH_IPOPT_SUPPORT		
} 



} // namespace example
} // namespace optcon
} // namespace ct

/*!
 * This unit test applies Direct Multiple Shooting to an oscillator system, uses different solvers and compares the outputs.
 * \example oscDMSTest.cpp
 */
int main(int argc, char **argv)
{
	using namespace ct::optcon::example;
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}


