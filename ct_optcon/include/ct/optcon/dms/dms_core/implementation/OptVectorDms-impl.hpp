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

/**
 * @file		dms.hpp
 * @author		Markus Giftthaler, Markus Stäuble
 * @date		21/11/2016
 * @version		0.1
 *
 * A "pair" denotes a pair of state vector s_i and control vector q_i in the optimization vector w.
 * Each pair has a number which identifies it uniquely.
 *
 */

namespace ct {
namespace optcon { 

template <size_t STATE_DIM, size_t CONTROL_DIM>
OptVectorDms<STATE_DIM, CONTROL_DIM>::OptVectorDms(size_t n, const DmsSettings& settings) :
		OptVector(n),
		settings_(settings),
		numPairs_(settings.N_+1),
		controlInputConstrained_(false),
		performWarmStart_(false),
		spliner_(nullptr)
{
	timeGrid_ = std::shared_ptr<TimeGrid> (new TimeGrid (settings.N_, settings.T_));

	/* set up the mappings from optimization variables to position indices in w.
	 * If an optimization variable is of vector-type itself, for example s_i,
	 * the position index points to its first element. */

	// the first part of the optimization vector is filled with s_i's and q_i's, with i = 0, 1, 2,..., N
	for (size_t i = 0; i< numPairs_; i++)
	{
		size_t s_index = i*(STATE_DIM+CONTROL_DIM);
		pairNumToStateIdx_.insert(std::make_pair(i, s_index));

		size_t q_index = s_index + STATE_DIM;
		pairNumToControlIdx_.insert(std::make_pair(i, q_index));
	}


	/* check if time is to be included as optimization variable*/
	if (settings_.objectiveType_ == DmsSettings::OPTIMIZE_GRID)
	{
		// N additional opt variables for h_0, h_1, ..., h_(N-1)
		for (size_t i = 0; i<settings_.N_; i++)
		{
			size_t h_index = numPairs_*(STATE_DIM + CONTROL_DIM) + i; // the index refers to the shot duration following a pair
			shotNumToShotDurationIdx_.insert(std::make_pair(i, h_index));
		}
	}

	setShotLowerBounds();

	// initialize the spliner for control inputs
	switch(settings_.splineType_)
	{
	case DmsSettings::ZERO_ORDER_HOLD:
	{
		spliner_ = std::shared_ptr<ZeroOrderHoldSpliner<control_vector_t>>(
				new ZeroOrderHoldSpliner<control_vector_t> (timeGrid_));
		break;
	}
	case DmsSettings::PIECEWISE_LINEAR:
	{
		spliner_ = std::shared_ptr<LinearSpliner<control_vector_t>>(
				new LinearSpliner<control_vector_t> (timeGrid_));
		break;
	}
	default:
	{
		throw(std::runtime_error("specified invalid spliner type in OptVectorDms-class"));
	}
	}
}

// This function is called on Observer update
template <size_t STATE_DIM, size_t CONTROL_DIM>
void OptVectorDms<STATE_DIM, CONTROL_DIM>::update()
{
	if(settings_.objectiveType_ == DmsSettings::OPTIMIZE_GRID)
	{
		const Eigen::VectorXd& h_segment =
				x_.segment(shotNumToShotDurationIdx_.find(0)->second, settings_.N_);

		timeGrid_->updateTimeGrid(h_segment);
	}

	splineControls();	
}


template <size_t STATE_DIM, size_t CONTROL_DIM>
const typename DmsDimensions<STATE_DIM, CONTROL_DIM>::state_vector_t OptVectorDms<STATE_DIM, CONTROL_DIM>::getState(const size_t pairNum)
{
	size_t index = pairNumToStateIdx_.find(pairNum)->second;
	return (x_.segment(index , STATE_DIM));
}

/* get a reference to the control corresponding to a desired pair number */
template <size_t STATE_DIM, size_t CONTROL_DIM>
const typename DmsDimensions<STATE_DIM, CONTROL_DIM>::control_vector_t OptVectorDms<STATE_DIM, CONTROL_DIM>::getControl(const size_t pairNum)
{
	size_t index = pairNumToControlIdx_.find(pairNum)->second;
	return (x_.segment(index, CONTROL_DIM));
}


template <size_t STATE_DIM, size_t CONTROL_DIM>
const typename DmsDimensions<STATE_DIM, CONTROL_DIM>::state_vector_array_t& OptVectorDms<STATE_DIM, CONTROL_DIM>::getStateSolution()
{
	stateSolution_.clear();
	for(size_t i = 0 ; i < numPairs_ ; i++ )
	{
		stateSolution_.push_back(getState(i));
	}
	return stateSolution_;
}


template <size_t STATE_DIM, size_t CONTROL_DIM>
const typename DmsDimensions<STATE_DIM, CONTROL_DIM>::control_vector_array_t& OptVectorDms<STATE_DIM, CONTROL_DIM>::getInputSolution()
{
	inputSolution_.clear();;
	for(size_t i = 0 ; i < numPairs_ ; i++ )
	{
		inputSolution_.push_back(getControl(i));
	}
	return inputSolution_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
const typename DmsDimensions<STATE_DIM, CONTROL_DIM>::time_array_t& OptVectorDms<STATE_DIM, CONTROL_DIM>::getTimeSolution()
{
	return timeGrid_->getTimeGrid();
}


template <size_t STATE_DIM, size_t CONTROL_DIM>
size_t OptVectorDms<STATE_DIM, CONTROL_DIM>::getStateIndex(const size_t pairNum)
{
	return pairNumToStateIdx_.find(pairNum)->second;
}


template <size_t STATE_DIM, size_t CONTROL_DIM>
size_t OptVectorDms<STATE_DIM, CONTROL_DIM>::getControlIndex(const size_t pairNum)
{
	return pairNumToControlIdx_.find(pairNum)->second;
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
size_t OptVectorDms<STATE_DIM, CONTROL_DIM>::getShotDurationIndex(const size_t shotNr)
{
	return (shotNumToShotDurationIdx_.find(shotNr)->second);
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
double OptVectorDms<STATE_DIM, CONTROL_DIM>::getShotStartTime(const size_t shotNr)
{
	return timeGrid_->getShotStartTime(shotNr);
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
double OptVectorDms<STATE_DIM, CONTROL_DIM>::getShotEndTime(const size_t shotNr)
{
	return timeGrid_->getShotEndTime(shotNr);
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
double OptVectorDms<STATE_DIM, CONTROL_DIM>::getShotDuration(const size_t shotNr)
{
	return timeGrid_->getShotDuration(shotNr);
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
double OptVectorDms<STATE_DIM, CONTROL_DIM>::getTtotal()
{
	return timeGrid_->getTtotal();
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
void OptVectorDms<STATE_DIM, CONTROL_DIM>::setInitGuess(const state_vector_t& x0, const state_vector_t& x_f, const control_vector_t& u0)
{
	size_t type = 1; // 0 = constant init guess, 1 = linearly interpolated between x0 and x_f

	// init the states s_i and controls q_i
	for (size_t i = 0; i < numPairs_; i++)
	{
		size_t s_index = pairNumToStateIdx_.find(i)->second;
		size_t q_index = pairNumToControlIdx_.find(i)->second;

		switch(type)
		{
		case 0:
		{
			xInitGuess_.segment(s_index , STATE_DIM) = x0;
			break;
		}
		case 1:
			xInitGuess_.segment(s_index , STATE_DIM) = x0+(x_f-x0)*(i/(numPairs_-1));
				break;
		}
		xInitGuess_.segment(q_index , CONTROL_DIM) = u0;
	}

	// initialize the time parameters
	if(settings_.objectiveType_ == DmsSettings::OPTIMIZE_GRID)
	{
		for (size_t i = 0; i< settings_.N_; i++)
		{
			size_t h_index = getShotDurationIndex(i);
			Eigen::Matrix<double, 1, 1> newElement;
			newElement(0,0) = (timeGrid_->getShotEndTime(i) - timeGrid_->getShotStartTime(i));
			xInitGuess_.segment(h_index, 1) = newElement;
		}
	}
	Base::setInitGuess();
}


template <size_t STATE_DIM, size_t CONTROL_DIM>
void OptVectorDms<STATE_DIM, CONTROL_DIM>::setInitGuess(
	const state_vector_array_t& x_init, 
	const control_vector_array_t& u_init, 
	const time_array_t& t_init)
{
	if(x_init.size() != numPairs_) throw std::runtime_error("initial guess state trajectory not matching number of shots");
	if(u_init.size() != numPairs_) throw std::runtime_error("initial guess input trajectory not matching number of shots");

	for (size_t i = 0; i < numPairs_; i++)
		{
			size_t s_index = pairNumToStateIdx_.find(i)->second;
			size_t q_index = pairNumToControlIdx_.find(i)->second;

			xInitGuess_.segment(s_index , STATE_DIM) = x_init[i];
			xInitGuess_.segment(q_index , CONTROL_DIM) = u_init[i];
		}

		// initialize the time parameters
		if(settings_.objectiveType_ == DmsSettings::OPTIMIZE_GRID)
		{
			for (size_t i = 0; i< settings_.N_; i++)
			{
				size_t h_index = getShotDurationIndex(i);
				Eigen::Matrix<double, 1, 1> newElement;
				newElement(0,0) = (timeGrid_->getShotEndTime(i) - timeGrid_->getShotStartTime(i));
				xInitGuess_.segment(h_index, 1) = newElement;
			}
		}
	Base::setInitGuess();
}


template <size_t STATE_DIM, size_t CONTROL_DIM>
Eigen::VectorXd OptVectorDms<STATE_DIM, CONTROL_DIM>::getInitGuess()
{
	if(performWarmStart_)
		return x_;
	else
		return xInitGuess_;
}


template <size_t STATE_DIM, size_t CONTROL_DIM>
void OptVectorDms<STATE_DIM, CONTROL_DIM>::updateInitialState(const state_vector_t& newInitialState)
{
	size_t s_index = pairNumToStateIdx_.find(0)->second;
	x_.segment(s_index , STATE_DIM) = newInitialState;
}


template <size_t STATE_DIM, size_t CONTROL_DIM>
void OptVectorDms<STATE_DIM, CONTROL_DIM>::updateDesiredState(const state_vector_t& newDesiredState)
{
	size_t s_index = pairNumToStateIdx_.find(settings_.N_)->second;
	x_.segment(s_index , STATE_DIM) = newDesiredState;
}


template <size_t STATE_DIM, size_t CONTROL_DIM>
void OptVectorDms<STATE_DIM, CONTROL_DIM>::setShotLowerBounds()
{
	if(settings_.objectiveType_ == DmsSettings::OPTIMIZE_GRID)
	{
		for (size_t i = 0; i< settings_.N_; i++)
		{
			size_t h_index = shotNumToShotDurationIdx_.find(i)->second;
			Eigen::Matrix<double, 1, 1> newElement;
			newElement(0,0) = settings_.h_min_ ;
			xLb_.segment(h_index, 1) = newElement; //0.0;
			newElement(0,0) = std::numeric_limits<double>::max();
			xUb_.segment(h_index, 1) = newElement;
		}
	}
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
void OptVectorDms<STATE_DIM, CONTROL_DIM>::splineControls()
{
	spliner_->computeSpline(getInputSolution().toImplementation());
}


template <size_t STATE_DIM, size_t CONTROL_DIM>
typename DmsDimensions<STATE_DIM, CONTROL_DIM>::control_vector_t OptVectorDms<STATE_DIM, CONTROL_DIM>::getControlFromSpline(const double time, const size_t shotIdx)
{
	control_vector_t result = spliner_->evalSpline(time, shotIdx);

	assert(result == result && "there is a NaN in the input from the spliner."); // assert that there's no NaN
	assert(result-result == (result-result) && "the input from the spliner is infinite");

	return result;
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
typename DmsDimensions<STATE_DIM, CONTROL_DIM>::control_vector_t OptVectorDms<STATE_DIM, CONTROL_DIM>::getControlFromSpline(const double time)
{
	control_vector_t result = spliner_->evalSpline(time, timeGrid_->getShotIdx(time));

	assert(result == result && "there is a NaN in the input from the spliner."); // assert that there's no NaN
	assert(result-result == (result-result) && "the input from the spliner is infinite");

	return result;
}


template <size_t STATE_DIM, size_t CONTROL_DIM>
typename DmsDimensions<STATE_DIM, CONTROL_DIM>::control_matrix_t OptVectorDms<STATE_DIM, CONTROL_DIM>::splineDerivative_q_i(const double time, const size_t shotIdx) const
{
	return (spliner_->splineDerivative_q_i(time, shotIdx));
}


template <size_t STATE_DIM, size_t CONTROL_DIM>
typename DmsDimensions<STATE_DIM, CONTROL_DIM>::control_matrix_t OptVectorDms<STATE_DIM, CONTROL_DIM>::splineDerivative_q_iplus1(const double time, const size_t shotIdx) const
{
	return (spliner_->splineDerivative_q_iplus1(time, shotIdx));
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
typename DmsDimensions<STATE_DIM, CONTROL_DIM>::control_vector_t OptVectorDms<STATE_DIM, CONTROL_DIM>::splineDerivative_t(const double time, const size_t shotIdx) const
{
	return spliner_->splineDerivative_t(time, shotIdx);
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
typename DmsDimensions<STATE_DIM, CONTROL_DIM>::control_vector_t OptVectorDms<STATE_DIM, CONTROL_DIM>::splineDerivative_h_i(const double time, const size_t shotIdx) const
{
	return spliner_->splineDerivative_h_i(time, shotIdx);
}


template <size_t STATE_DIM, size_t CONTROL_DIM>
void OptVectorDms<STATE_DIM, CONTROL_DIM>::printoutSolution()
{
	std::cout << "... printing solutions: " << std::endl;
	std::cout << "x_solution"<< std::endl;
	state_vector_array_t x_sol = getStateSolution();
	for(size_t i =0; i<x_sol.size(); ++i){
		std::cout << x_sol[i].transpose() << std::endl;
	}

	std::cout << "u_solution"<< std::endl;
	control_vector_array_t u_sol = getInputSolution();
	for(size_t i =0; i<u_sol.size(); ++i){
		std::cout << u_sol[i].transpose() << std::endl;
	}

	std::cout << "t_solution"<< std::endl;
	const time_array_t& t_sol = timeGrid_->getTimeGrid();
	for(size_t i =0; i<t_sol.size(); ++i){
		std::cout << t_sol[i] << "  ";
	}
	std::cout << std::endl;
	std::cout << " ... done." << std::endl;
}

} // namespace optcon
} // namespace ct
