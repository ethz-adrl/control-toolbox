/*
 * MPC_DMS.hpp
 *
 *      Author: kusi
 */

 #include <chrono>


template <size_t STATE_DIM, size_t CONTROL_DIM>
void MpcDms<STATE_DIM, CONTROL_DIM>::replanTrajectory(
	const state_vector_t& currState,
	const ct::core::Time currTime
	)
{
	state_vector_t startState;
	if(settingsReplan_.stateForwardIntegration_)
	{
		std::chrono::microseconds delayUs(0);
		delayUs = std::chrono::microseconds(static_cast<int>(settingsReplan_.fixedDelayUs_));
		rollout(currState, startState, delayUs.count());
	}
	else
		startState = currState;

#ifdef DEBUG_ILQG_MPC
	std::cout << "planning from joint state ..." << std::endl;
	std::cout << startState.transpose() << std::endl;
#endif

	if(settingsReplan_.shrinkTimeHorizon_)
		shiftInitGuess(startState, currTime);
	else
		updateInitGuess(startState, currTime);

	my_dms_->updateInitialState(startState);
	my_dms_->setInitGuess(xInitguess_, uInitguess_);
	solverReplanning_->UpdateInstance();
	solverReplanning_->UpdateInitGuess(); //works better now
	solverReplanning_->Solve();

	// for(size_t i = 0; i < previousX_; ++i)
	// {
	// 	std::cout << "state: " << previousX_[i] << std::endl;
	// }

	//Update the previous trajectories
	my_dms_->retrieveStateVectorArray(previousX_);
	my_dms_->retrieveInputTrajectory(previousU_);
	previousT_.swap(newT_);
	lastTime_ = currTime;

	// exportTrajectories();

}

// template <size_t STATE_DIM, size_t CONTROL_DIM>
// void MpcDms<STATE_DIM, CONTROL_DIM>::exportTrajectories()
// {
// 	i++;
// 	std::cout << "calling exportTrajectories" << std::endl;
// 	std::string matlabPath = "/home/markusta/Documents/Code/catkin_ws/src/ct_hya_dms/matfiles/test" + std::to_string(i) + ".mat";
// 	MatlabInterface mi(matlabPath);
// 	mi.sendMultiDimTrajectoryToMatlab<state_vector_array_t>(xInitguess_, 12, "x_init");
// 	mi.sendMultiDimTrajectoryToMatlab<control_vector_array_t>(uInitguess_, 6, "u_init");
// 	mi.sendMultiDimTrajectoryToMatlab<state_vector_array_t>(this->previousX_, 12, "x_opt");
// 	mi.sendMultiDimTrajectoryToMatlab<control_vector_array_t>(this->previousU_, 6, "u_opt");
// 	mi.sendScalarArrayToMatlab(this->previousT_, "t_opt");

// 	// std::string resultDir = "/home/markusta/Documents/Code/catkin_ws/src/ct_hya_dms/xml/test";
// 	// std::string stateInitguessFile = resultDir + "/initGuessState" + std::to_string(i) + ".xml";
// 	// std::string inputInitguessFile = resultDir + "/initGuessInput" + std::to_string(i) + ".xml";
// 	// std::string stateSolutionFile = resultDir + "/solutionState" + std::to_string(i) + ".xml";
// 	// std::string inputSolutionFile = resultDir + "/solutionInput" + std::to_string(i) + ".xml";
// 	// std::string timeSolutionFile = resultDir + "/solutionTime" + std::to_string(i) + ".xml";
// 	// {
// 	// 	std::ofstream xmlInitguessState(stateInitguessFile);
// 	// 	cereal::XMLOutputArchive archive_initState_xml(xmlInitguessState);
// 	// 	archive_initState_xml(CEREAL_NVP(xInitguess_));

// 	// 	std::ofstream xmlInitguessInput(inputInitguessFile);
// 	// 	cereal::XMLOutputArchive archive_initInput_xml(xmlInitguessInput);
// 	// 	archive_initInput_xml(CEREAL_NVP(uInitguess_));

// 	// 	std::ofstream xmlStateSolution(stateSolutionFile);
// 	// 	cereal::XMLOutputArchive archive_solutionState_xml(xmlStateSolution);
// 	// 	archive_solutionState_xml(CEREAL_NVP(this->previousX_));

// 	// 	std::ofstream xmlInputSolution(inputSolutionFile);
// 	// 	cereal::XMLOutputArchive archive_solutionInput_xml(xmlInputSolution);
// 	// 	archive_solutionInput_xml(CEREAL_NVP(this->previousU_));

// 	// 	std::ofstream xmlTimeSolution(timeSolutionFile);
// 	// 	cereal::XMLOutputArchive archive_solutionTime_xml(xmlTimeSolution);
// 	// 	archive_solutionTime_xml(CEREAL_NVP(this->previousT_));
// 	// }
// }

template <size_t STATE_DIM, size_t CONTROL_DIM>
void MpcDms<STATE_DIM, CONTROL_DIM>::getAnimation(
			const double dtInt,
			state_vector_array_t& x_animation,
			control_vector_array_t& u_animation,
			time_array_t& t_animation)
{
		my_dms_->retrieveAnimation(dtInt, x_animation, u_animation, t_animation);
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
void MpcDms<STATE_DIM, CONTROL_DIM>::getSolution(
	state_vector_array_t& x_solution, 
	control_vector_array_t& u_solution, 
	time_array_t& t_solution)
{
		my_dms_->retrieveStateVectorArray(x_solution);
		my_dms_->retrieveInputTrajectory(u_solution);
		my_dms_->retrieveTimeArray(t_solution);
}

//Currently not in use
// Used for integrating measurement when dealing with delay
template <size_t STATE_DIM, size_t CONTROL_DIM>
void MpcDms<STATE_DIM, CONTROL_DIM>::rollout(
		const state_vector_t& stateInConst, 
		state_vector_t& stateOut,
		const double delayUs)
{
	assert(K_traj_.size() > 0);
	assert(u_ff_traj_.size() > 0);
	state_vector_t stateIn = stateInConst;

	state_vector_array_t statesOut;
	size_t control_steps = round(delayUs / settingsReplan_.dt_control_ / 1000 / 1000);
	statesOut.resize(control_steps + 1);

	std::shared_ptr<ct::core::StateFeedbackController<STATE_DIM, CONTROL_DIM>> stateFeedbackCtrl (new
		ct::core::StateFeedbackController<STATE_DIM, CONTROL_DIM> (u_ff_traj_, K_traj_, settingsReplan_.dt_control_));

	dynamics_->setController(stateFeedbackCtrl);

	// perform rollout of stabilized solution
	time_array_t t_temp;
	integrator_->integrate_n_steps(stateIn, 0.0, control_steps, settingsReplan_.dt_control_, statesOut, t_temp);
	stateOut = statesOut.back();
}


template<size_t STATE_DIM, size_t CONTROL_DIM>
void MpcDms<STATE_DIM, CONTROL_DIM>::updateInitGuess(const state_vector_t& startState, const ct::core::Time currTime)
{
	xInitguess_ = previousX_;
	uInitguess_ = previousU_;
	xInitguess_.front() = startState;
}


template<size_t STATE_DIM, size_t CONTROL_DIM>
void MpcDms<STATE_DIM, CONTROL_DIM>::shiftInitGuess(const state_vector_t& startState, const ct::core::Time currTime)
{
	stateSpliner_->genSpline(previousX_, previousT_);
	controlSpliner_->genSpline(previousU_, previousT_);
	double oldTBack = previousT_.back();

	//Set new timehorizon
	timeHorizon_ = settingsDms_.T_ - currTime - settingsReplan_.fixedDelayUs_ / 1000 / 1000;

	my_dms_->updateTimeHorizon(timeHorizon_);
	my_dms_->retrieveTimeArray(newT_);

	//Initguess
	// delta T between to DMS iterations
	double deltaT = currTime - lastTime_;

	xInitguess_.clear();
	uInitguess_.clear();
	xInitguess_.push_back(startState);
	uInitguess_.push_back(controlSpliner_->evalSpline(deltaT));

	// std::cout << "deltaT: " << deltaT << std::endl;

	// Make sure the init guess is "smooth" -> SNOPT will converge faster
	for(size_t i = 1; i < settingsDms_.N_; ++i)
	{
		double tEval = newT_[i] + deltaT + settingsReplan_.fixedDelayUs_ / 1000 / 1000;
		//This if else should vanish after testing
		if(tEval <= oldTBack)
		{
			xInitguess_.push_back(stateSpliner_->evalSpline(tEval));
			uInitguess_.push_back(controlSpliner_->evalSpline(tEval));
		}

		//we only enter here, when dealing with delay
		else
		{
			// std::cout << "This should rarely happen" << std::endl;
			xInitguess_.push_back(terminalState_);
			uInitguess_.push_back(previousU_.back());
		}
	}

	xInitguess_.push_back(terminalState_);
	uInitguess_.push_back(previousU_.back());
}