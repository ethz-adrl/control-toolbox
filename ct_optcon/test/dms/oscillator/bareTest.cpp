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

#include <ct/optcon/optcon.h>

using namespace ct;
using namespace core;

namespace ct{
namespace optcon{
namespace example{


class OscillatorDms
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef CppAD::AD<CppAD::cg::CG<double> > ScalarCG;

	typedef DmsDimensions<2,1> OscDimensions;

	OscillatorDms() :
		w_n_(0.5),
		zeta_(0.01)
	{}

	~OscillatorDms(){
		std::cout << "Oscillator dms destructor called" << std::endl;
	}


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

		stateInputConstraints_ = std::shared_ptr<ct::optcon::ConstraintContainerAnalytical<2,1>>
				(new ct::optcon::ConstraintContainerAnalytical<2,1>() );

		stateInputConstraintsAd_ = std::shared_ptr<ct::optcon::ConstraintContainerAD<2,1>>
				(new ct::optcon::ConstraintContainerAD<2,1>());

		pureStateConstraints_ = std::shared_ptr<ct::optcon::ConstraintContainerAnalytical<2, 1>>
				(new ct::optcon::ConstraintContainerAnalytical<2, 1>());

		std::shared_ptr<TerminalConstraint<2,1>> termConstraint(new TerminalConstraint<2,1>(x_final_));
		core::StateVector<2> xLow;
		core::StateVector<2> xHigh;
		xLow << -5.0, -5.0;
		xHigh << 5.0, 5.0; 

		core::ControlVector<1> uLow;
		core::ControlVector<1> uHigh;
		uLow << -10.0;
		uHigh << 10.0; 

		std::shared_ptr<ControlInputConstraint<2,1>> inputConstraint(new ControlInputConstraint<2,1>(uLow, uHigh));
		std::shared_ptr<StateConstraint<2,1>> stateConstraint(new StateConstraint<2,1>(xLow, xHigh));

		std::shared_ptr<optcon::tpl::StateConstraint<2, 1, ScalarCG>> stateConstraintAd(
			new optcon::tpl::StateConstraint<2, 1, ScalarCG>(xLow, xHigh));
		std::shared_ptr<optcon::tpl::ControlInputConstraint<2, 1, ScalarCG>> inputConstraintAd(
			new optcon::tpl::ControlInputConstraint<2, 1, ScalarCG> (uLow, uHigh));
		std::shared_ptr<optcon::tpl::TerminalConstraint<2,1, ScalarCG>> termConstraintAd(
			new optcon::tpl::TerminalConstraint<2,1, ScalarCG>(x_final_));

		termConstraint->setName("TerminalConstraint");
		stateConstraint->setName("StateConstraint");
		inputConstraint->setName("ControlInputConstraint");
	
		stateConstraintAd->setName("StateConstraintAd");
		inputConstraintAd->setName("InputConstraintAd");
		termConstraintAd->setName("TerminalConstraintAd");

		stateInputConstraintsAd_->addIntermediateConstraint(stateConstraintAd, true);
		stateInputConstraintsAd_->addIntermediateConstraint(inputConstraintAd, true);
		stateInputConstraintsAd_->addTerminalConstraint(termConstraintAd, true);

		pureStateConstraints_->addTerminalConstraint(termConstraint, true);
		stateInputConstraints_->addIntermediateConstraint(inputConstraint, true);
		stateInputConstraints_->addIntermediateConstraint(stateConstraint, true);

		OptConProblem<2,1> optProblem(oscillator_, costFunction_);
		optProblem.setInitialState(x_0_);
		optProblem.setTimeHorizon(settings_.T_);
		optProblem.setStateInputConstraints(stateInputConstraints_);
		optProblem.setPureStateConstraints(pureStateConstraints_);

		calcInitGuess();
		dmsPlanner_ = std::shared_ptr<DmsSolver<2,1>> (new DmsSolver<2,1>(optProblem, settings_));
		dmsPlanner_->setInitialGuess(initialPolicy_);
	}

	void getSolution()
	{
		dmsPlanner_->solve();
	}



private:
	void calcInitGuess()
	{
		x_initguess_.resize(settings_.N_ + 1, OscDimensions::state_vector_t::Zero());
		u_initguess_.resize(settings_.N_ + 1, OscDimensions::control_vector_t::Zero());
		for(size_t i = 0; i < settings_.N_ + 1; ++i)
		{
			x_initguess_[i] = x_0_ + (x_final_ - x_0_) * ((double)i /(double) settings_.N_);
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
	std::shared_ptr<ct::optcon::ConstraintContainerAnalytical<2,1> > stateInputConstraints_;
	std::shared_ptr<ct::optcon::ConstraintContainerAD<2,1> > stateInputConstraintsAd_;
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

void runTests()
{
	for(int solverType = 0; solverType < NlpSolverSettings::SolverType::num_types_solver; solverType++)
	{
		int splineType = 1;
		int costEvalT = 1;
		int optGrid = 0;
		int integrateSensitivity = 1;
		
		// have to manually exclude the following case, which is not implemented
		if (integrateSensitivity == 0 && costEvalT == DmsSettings::FULL)
			continue;

		DmsSettings settings;
		settings.N_ = 25;
		settings.T_ = 5.0;
		settings.nThreads_ = 4;
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

		NlpSolverSettings nlpsettings;
		nlpsettings.solverType_ = static_cast<NlpSolverSettings::SolverType>(solverType);	// IPOPT, SNOPT
		settings.nlpSettings_ = nlpsettings;

		OscillatorDms oscDms;
		oscDms.initialize(settings);

		oscDms.getSolution();
	}
}

} // namespace example
} // namespace optcon
} // namespace ct

int main(int argc, char **argv)
{
	using namespace ct::optcon::example;
	runTests();
	return 1;
}
