/***********************************************************************************
Copyright (c) 2017, Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo,
Farbod Farshidian. All rights reserved.

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

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/

#pragma once

#include <boost/numeric/odeint.hpp>
#include "SteppersODEIntDefinitions.h"

namespace ct {
namespace core {
namespace internal {

/**
 * @brief      The interface to call the integration routines from ODEInt
 *
 * @tparam     STEPPER  The ODEInt stepper
 * @tparam     MATRIX   The matrix type
 * @tparam     SCALAR   The scalar type
 */
template <class STEPPER, typename MATRIX, typename SCALAR = double>
class StepperODEInt : public StepperBase<MATRIX, SCALAR>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	StepperODEInt() {}
	virtual void integrate_n_steps(const std::function<void(const MATRIX&, MATRIX&, SCALAR)>& rhs,
		MATRIX& state,
		const SCALAR& startTime,
		size_t numSteps,
		SCALAR dt) override
	{
		boost::numeric::odeint::integrate_n_steps(stepper_, rhs, state, startTime, dt, numSteps);
	}

	virtual void integrate_n_steps(std::function<void(const MATRIX& x, const SCALAR& t)> observer,
		const std::function<void(const MATRIX&, MATRIX&, SCALAR)>& rhs,
		MATRIX& state,
		const SCALAR& startTime,
		size_t numSteps,
		SCALAR dt) override
	{
		boost::numeric::odeint::integrate_n_steps(stepper_, rhs, state, startTime, dt, numSteps, observer);
	}


	virtual void integrate_const(const std::function<void(const MATRIX&, MATRIX&, SCALAR)>& rhs,
		MATRIX& state,
		const SCALAR& startTime,
		const SCALAR& finalTime,
		SCALAR dt) override
	{
		boost::numeric::odeint::integrate_const(stepper_, rhs, state, startTime, finalTime, dt);
	}

	virtual void integrate_const(std::function<void(const MATRIX& x, const SCALAR& t)> observer,
		const std::function<void(const MATRIX&, MATRIX&, SCALAR)>& rhs,
		MATRIX& state,
		const SCALAR& startTime,
		const SCALAR& finalTime,
		SCALAR dt) override
	{
		boost::numeric::odeint::integrate_const(stepper_, rhs, state, startTime, finalTime, dt, observer);
	}

	virtual void integrate_adaptive(const std::function<void(const MATRIX&, MATRIX&, SCALAR)>& rhs,
		MATRIX& state,
		const SCALAR& startTime,
		const SCALAR& finalTime,
		SCALAR dtInitial = SCALAR(0.01)) override
	{
		boost::numeric::odeint::integrate_adaptive(stepper_, rhs, state, startTime, finalTime, dtInitial);
	}

	virtual void integrate_adaptive(std::function<void(const MATRIX& x, const SCALAR& t)> observer,
		const std::function<void(const MATRIX&, MATRIX&, SCALAR)>& rhs,
		MATRIX& state,
		const SCALAR& startTime,
		const SCALAR& finalTime,
		const SCALAR dtInitial = SCALAR(0.01)) override
	{
		boost::numeric::odeint::integrate_adaptive(stepper_, rhs, state, startTime, finalTime, dtInitial, observer);
	}

	virtual void integrate_times(std::function<void(const MATRIX& x, const SCALAR& t)> observer,
		const std::function<void(const MATRIX&, MATRIX&, SCALAR)>& rhs,
		MATRIX& state,
		const tpl::TimeArray<SCALAR>& timeTrajectory,
		SCALAR dtInitial = SCALAR(0.01)) override
	{
		boost::numeric::odeint::integrate_times(
			stepper_, rhs, state, &timeTrajectory.front(), &timeTrajectory.back() + 1, dtInitial, observer);
	}

private:
	STEPPER stepper_;
};


/**
 * @brief      The interface to call ODEInt Dense Output Integration routines
 *
 * @tparam     STEPPER  The ODEInt stepper
 * @tparam     MATRIX   The matrix type
 * @tparam     SCALAR   The scalar type
 */
template <class STEPPER, typename MATRIX, typename SCALAR = double>
class StepperODEIntDenseOutput : public StepperODEInt<STEPPER, MATRIX, SCALAR>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	typedef typename boost::numeric::odeint::result_of::make_dense_output<STEPPER>::type StepperDense;

	StepperODEIntDenseOutput()
	{
		stepperDense_ = boost::numeric::odeint::make_dense_output(this->absErrTol_, this->relErrTol_, stepper_);
	}

	virtual void integrate_adaptive(const std::function<void(const MATRIX&, MATRIX&, SCALAR)>& rhs,
		MATRIX& state,
		const SCALAR& startTime,
		const SCALAR& finalTime,
		SCALAR dtInitial = SCALAR(0.01)) override
	{
		stepperDense_.initialize(state, startTime, dtInitial);
		boost::numeric::odeint::integrate_adaptive(stepperDense_, rhs, state, startTime, finalTime, dtInitial);
	}

	virtual void integrate_adaptive(std::function<void(const MATRIX& x, const SCALAR& t)> observer,
		const std::function<void(const MATRIX&, MATRIX&, SCALAR)>& rhs,
		MATRIX& state,
		const SCALAR& startTime,
		const SCALAR& finalTime,
		const SCALAR dtInitial = SCALAR(0.01)) override
	{
		stepperDense_.initialize(state, startTime, dtInitial);
		boost::numeric::odeint::integrate_adaptive(
			stepperDense_, rhs, state, startTime, finalTime, dtInitial, observer);
	}

	virtual void integrate_times(std::function<void(const MATRIX& x, const SCALAR& t)> observer,
		const std::function<void(const MATRIX&, MATRIX&, SCALAR)>& rhs,
		MATRIX& state,
		const tpl::TimeArray<SCALAR>& timeTrajectory,
		SCALAR dtInitial = SCALAR(0.01)) override
	{
		stepperDense_.initialize(state, timeTrajectory.front(), dtInitial);
		boost::numeric::odeint::integrate_times(
			stepperDense_, rhs, state, &timeTrajectory.front(), &timeTrajectory.back() + 1, dtInitial, observer);
	}

private:
	STEPPER stepper_;
	StepperDense stepperDense_;
};

/**
 * @brief      The interface to call ODEInt Controlled integration routines
 *
 * @tparam     STEPPER  The ODEInt stepper type
 * @tparam     MATRIX   The Matrix Type
 * @tparam     SCALAR   The Scalar
 */
template <class STEPPER, typename MATRIX, typename SCALAR = double>
class StepperODEIntControlled : public StepperODEInt<STEPPER, MATRIX, SCALAR>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	typedef typename boost::numeric::odeint::result_of::make_controlled<STEPPER>::type StepperControlled;

	StepperODEIntControlled()
	{
		stepperControlled_ = boost::numeric::odeint::make_controlled(this->absErrTol_, this->relErrTol_, stepper_);
	}

	virtual void integrate_adaptive(const std::function<void(const MATRIX&, MATRIX&, SCALAR)>& rhs,
		MATRIX& state,
		const SCALAR& startTime,
		const SCALAR& finalTime,
		SCALAR dtInitial = SCALAR(0.01)) override
	{
		boost::numeric::odeint::integrate_adaptive(stepperControlled_, rhs, state, startTime, finalTime, dtInitial);
	}

	virtual void integrate_adaptive(std::function<void(const MATRIX& x, const SCALAR& t)> observer,
		const std::function<void(const MATRIX&, MATRIX&, SCALAR)>& rhs,
		MATRIX& state,
		const SCALAR& startTime,
		const SCALAR& finalTime,
		const SCALAR dtInitial = SCALAR(0.01)) override
	{
		boost::numeric::odeint::integrate_adaptive(
			stepperControlled_, rhs, state, startTime, finalTime, dtInitial, observer);
	}

	virtual void integrate_times(std::function<void(const MATRIX& x, const SCALAR& t)> observer,
		const std::function<void(const MATRIX&, MATRIX&, SCALAR)>& rhs,
		MATRIX& state,
		const tpl::TimeArray<SCALAR>& timeTrajectory,
		SCALAR dtInitial = SCALAR(0.01)) override
	{
		boost::numeric::odeint::integrate_times(
			stepperControlled_, rhs, state, &timeTrajectory.front(), &timeTrajectory.back() + 1, dtInitial, observer);
	}

private:
	STEPPER stepper_;
	StepperControlled stepperControlled_;
};
}
}
}
