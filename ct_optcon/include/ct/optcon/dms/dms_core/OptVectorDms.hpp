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

#ifndef CT_OPTCON_DMS_OPT_VECTOR_HPP_
#define CT_OPTCON_DMS_OPT_VECTOR_HPP_


#include <Eigen/Core>
#include <map>

#include <ct/core/core.h>

#include "TimeGrid.hpp"

#include <ct/optcon/dms/dms_core/DmsDimensions.hpp>
#include <ct/optcon/dms/dms_core/spline/SplinerBase.hpp>
#include <ct/optcon/dms/dms_core/spline/ZeroOrderHold/ZeroOrderHoldSpliner.hpp>
#include <ct/optcon/dms/dms_core/spline/Linear/LinearSpliner.hpp>
#include <ct/optcon/nlp/OptVector.h>
#include <ct/optcon/dms/dms_core/DmsSettings.hpp>


namespace ct {
namespace optcon {


template <size_t STATE_DIM, size_t CONTROL_DIM>
class OptVectorDms : public OptVector
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef DmsDimensions<STATE_DIM, CONTROL_DIM> DIMENSIONS;
	typedef OptVector Base;

	typedef typename DIMENSIONS::state_vector_t state_vector_t;
	typedef typename DIMENSIONS::control_vector_t control_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
	typedef typename DIMENSIONS::control_vector_array_t control_vector_array_t;
	typedef typename DIMENSIONS::control_matrix_t control_matrix_t;
	typedef typename DIMENSIONS::time_array_t time_array_t;

	OptVectorDms(size_t n, const DmsSettings& settings);

	// explicitly delete the copy constructor, since there's only one instance of OptVectorDms allowed
	OptVectorDms(const OptVectorDms& arg) = delete;

	~OptVectorDms(){}

	/* get the state from desired pair number */
	// const Eigen::VectorBlock<const state_vector_t, STATE_DIM> getState(const size_t pairNum)
	const state_vector_t getState(const size_t pairNum);

	/* get a reference to the control corresponding to a desired pair number */
	const control_vector_t getControl(const size_t pairNum);

	// get std::vector of references to all subsequent states in w
	const state_vector_array_t& getStateSolution();


	// get std::vector of references to all subsequent control input vectors in w
	const control_vector_array_t& getInputSolution();

	const time_array_t& getTimeSolution();

	// return starting index in w of the state of a particular pair
	size_t getStateIndex(const size_t pairNum);

	// return starting index in w of the control of a particular pair
	size_t getControlIndex(const size_t pairNum);

	/* return index in w of the time parameter of a particular pair
	 *
	 * get index of the time variable which defines the length of the shot
	 * following this particular pair.
	 * Example: for pair 0, which is located at t_0, this function returns
	 * the index of t_1
	 * */
	size_t getShotDurationIndex(const size_t shotNr);

	double getShotStartTime(const size_t shotNr);

	double getShotEndTime(const size_t shotNr);

	double getShotDuration(const size_t shotNr);

	double getTtotal();


	/* set each state in w to the same, identical x0 and
	 * set each contorl in w to the same, identical u0.
	 * */
	void setInitGuess(const state_vector_t& x0, const state_vector_t& x_f, const control_vector_t& u0);

	void setInitGuess(const state_vector_array_t& x_init, const control_vector_array_t& u_init, const time_array_t& t_init = time_array_t(0));


	// method that allows the solver to retrieve the initial guess.
	Eigen::VectorXd getInitGuess();


	/* method that allows to tweak the given initial state, use for WARMSTARTING */
	void updateInitialState(const state_vector_t& newInitialState);


	/* method that allows to tweak the desired terminal state, use for WARMSTARTING */
	void updateDesiredState(const state_vector_t& newDesiredState);


	/* set upper bound on times and set lower bound to 0.0 by default
	 * note: currently it is required to specify this.
	 * recently: setTimeHorizonUpperBound*/
	void setShotLowerBounds();


	void splineControls();


	control_vector_t getControlFromSpline(const double time, const size_t shotIdx);

	control_vector_t getControlFromSpline(const double time);

	size_t new_w_count() const
	{
		return getNewXCounter();
	}


	/* this method calculates the derivative of the spline w.r.t. the leading q_i in the optimization vector */
	control_matrix_t splineDerivative_q_i(const double time, const size_t shotIdx) const;

	/* this method calculates the derivative of the spline w.r.t. the following q, thus q_(i+1) in the optimization vector */
	control_matrix_t splineDerivative_q_iplus1(const double time, const size_t shotIdx) const;

	control_vector_t splineDerivative_t(const double time, const size_t shotIdx) const;

	control_vector_t splineDerivative_h_i(const double time, const size_t shotIdx) const;

	void activateWarmStart(){performWarmStart_ = true;}
	void deactivateWarmStart(){performWarmStart_ = false;}

	bool controlInputConstrained() const {return controlInputConstrained_;}

	size_t numPairs(){return numPairs_;}

	// print solution in console
	void printoutSolution();

	void update();

private:
	DmsSettings settings_;

	size_t numPairs_;
	bool controlInputConstrained_;
	bool performWarmStart_;

	/* splining */
	std::shared_ptr<SplinerBase<control_vector_t>> spliner_;
	std::shared_ptr<TimeGrid> timeGrid_;

	/* maps the number of a "pair" to the index in w where ... */
	std::map<size_t, size_t> pairNumToStateIdx_;			/* ... its state starts */
	std::map<size_t, size_t> pairNumToControlIdx_;  		/* ... its control starts */
	std::map<size_t, size_t> shotNumToShotDurationIdx_;		/* its shot time starts is in w (last element doesn't have one) */

	state_vector_array_t stateSolution_;
	control_vector_array_t inputSolution_;
	time_array_t timeSolution_;

};

} // namespace optcon
} // namespace ct

#include "implementation/OptVectorDms-impl.hpp"
#endif // CT_OPTCON_DMS_OPT_VECTOR_HPP_
