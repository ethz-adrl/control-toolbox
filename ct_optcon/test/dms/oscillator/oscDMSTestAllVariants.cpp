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

namespace ct{
namespace optcon{
namespace example{

class OscillatorDms
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef DmsDimensions<2,1> OscDimensions;

	OscillatorDms() :
		w_n_(0.5),
		zeta_(0.01)
	{}

	~OscillatorDms(){}


	void initialize(const DmsSettings settings)
	{
		settings_ = settings;
		settings_.print();

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


		pureStateConstraints_ = std::shared_ptr<ct::optcon::ConstraintContainerAnalytical<2, 1>>
				(new ct::optcon::ConstraintContainerAnalytical<2, 1>());

		std::shared_ptr<TerminalConstraint<2,1>> termConstraint(new TerminalConstraint<2,1>(x_final_));

		termConstraint->setName("crazyTerminalConstraint");

		pureStateConstraints_->addTerminalConstraint(termConstraint, true);

		pureStateConstraints_->initialize();

		OptConProblem<2,1> optProblem(oscillator_, costFunction_);
		optProblem.setInitialState(x_0_);
		optProblem.setTimeHorizon(settings_.T_);
		optProblem.setPureStateConstraints(pureStateConstraints_);

		calcInitGuess();
		dmsPlanner_ = std::shared_ptr<DmsSolver<2,1>> (new DmsSolver<2,1>(optProblem, settings_));
		dmsPlanner_->setInitialGuess(initialPolicy_);

	}

	void getSolution()
	{
		dmsPlanner_->solve();
		// dmsPlanner_->printSolution();
	}



private:

	void calcInitGuess()
	{
		x_initguess_.resize(settings_.N_ + 1, OscDimensions::state_vector_t::Zero());
		u_initguess_.resize(settings_.N_ + 1, OscDimensions::control_vector_t::Zero());
		for(size_t i = 0; i < settings_.N_ + 1; ++i)
		{
			x_initguess_[i] = x_0_ + (x_final_ - x_0_) * ((double)i / (double)settings_.N_);
		}

		initialPolicy_.xSolution_ = x_initguess_;
		initialPolicy_.uSolution_ = u_initguess_;
	}

	double w_n_;
	double zeta_;
	std::shared_ptr<ct::core::SecondOrderSystem > oscillator_;

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
	OscDimensions::state_vector_array_t x_initguess_;
	OscDimensions::control_vector_array_t u_initguess_;	
};

TEST(DmsTest, OscillatorDmsTestAllVariants)
{
	for(int splineType = 0; splineType < DmsSettings::SplineType::num_types_splining; splineType++)
	{
		for(int costEvalT = 0; costEvalT < DmsSettings::CostEvaluationType::num_types_costevaluation; costEvalT++)
		{
			for(int optGrid = 0; optGrid < DmsSettings::ObjectiveType::num_types_objectives; optGrid++)
			{
				for(int integrateSensitivity = 0; integrateSensitivity < 2; integrateSensitivity++)
				{

						// have to manually exclude the following case, which is not implemented
						if (integrateSensitivity == 0 && costEvalT == DmsSettings::FULL)
							continue;

						DmsSettings settings;
						settings.N_ = 25;
						settings.T_ = 5.0;
						settings.nThreads_ = 1;
						settings.terminalStateConstraint_ = 1;
						settings.splineType_ = static_cast<DmsSettings::SplineType>(splineType);	// ZOH, PWL
						settings.costEvaluationType_ =  static_cast<DmsSettings::CostEvaluationType>(costEvalT);	// SIMPLE, FULL
						settings.objectiveType_ = static_cast<DmsSettings::ObjectiveType>(optGrid);	// keep grid, opt. grid
						settings.h_min_ = 0.1;
						settings.integrationType_ = DmsSettings::RK4;
						settings.dt_sim_ = 0.01;
						settings.integrateSens_ =  static_cast<DmsSettings::IntegrationType>(integrateSensitivity);
						settings.absErrTol_ = 1e-6;
						settings.relErrTol_ = 1e-6;



#ifdef BUILD_WITH_SNOPT_SUPPORT
							NlpSolverSettings nlpsettings;
							nlpsettings.solverType_ = static_cast<NlpSolverSettings::SolverType>(0);	// IPOPT, SNOPT
							settings.nlpSettings_ = nlpsettings;
							OscillatorDms oscDms;
							oscDms.initialize(settings);
							oscDms.getSolution();
#endif

#ifdef BUILD_WITH_IPOPT_SUPPORT
							NlpSolverSettings nlpsettings_ipopt;
							nlpsettings_ipopt.solverType_ = static_cast<NlpSolverSettings::SolverType>(1);	// IPOPT, SNOPT
							settings.nlpSettings_ = nlpsettings_ipopt;
							OscillatorDms oscDms_ipopt;
							oscDms_ipopt.initialize(settings);
							oscDms_ipopt.getSolution();
#endif		
				}
			}
		}
	}
}


} // namespace example
} // namespace optcon
} // namespace ct


/*!
 * This unit test applies Direct Multiple Shooting to an oscillator system. It employs all possible combinations of settings and solvers.
 * \example oscDMSTestAllVariants.cpp
 */
int main(int argc, char **argv)
{
	using namespace ct::optcon::example;
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

