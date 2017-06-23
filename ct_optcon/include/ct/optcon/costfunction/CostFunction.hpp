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

#ifndef CT_OPTCON_COSTFUNCTION_HPP_
#define CT_OPTCON_COSTFUNCTION_HPP_

#include <vector>
#include <ct/core/core.h>

namespace ct {
namespace optcon {

/** \defgroup CostFunction CostFunction
 *
 * \brief Cost Function Module used in Optimal Control
 */

/**
 * \ingroup CostFunction
 *+
 * \brief A base function for cost functions. All cost functions should derive from this.
 *
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class CostFunction
{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef core::StateVector<STATE_DIM, SCALAR> state_vector_t;
	typedef core::ControlVector<CONTROL_DIM, SCALAR> control_vector_t;

	/**
	 * \brief Default constructor
	 *
	 * Default constructor, sets state, control and time to zero
	 */
	CostFunction() :
		t_(0.0),
		t_shift_(0.0)
	{
		x_.setZero();
		u_.setZero();
	};

	/**
	 * \brief Destructor
	 *
	 * Destructor
	 */
	virtual ~CostFunction() {};

	/**
	 * \brief Copy constructor
	 * @param arg other cost function
	 */
	CostFunction(const CostFunction& arg) :
		x_(arg.x_),
		u_(arg.u_),
		t_(arg.t_),
		t_shift_(arg.t_shift_)
	{}

	/**
	 * Clones the cost function.
	 * @return Base pointer to the clone
	 */
 	virtual CostFunction<STATE_DIM, CONTROL_DIM, SCALAR>* clone () const = 0;

	/**
	 * Set the current state, control and time of the cost function. In this function, the user can add pre-computations
	 * shared between different calls to the derivative functions.
	 * @param x state vector
	 * @param u control vector
	 * @param t time
	 */
	virtual void setCurrentStateAndControl(const state_vector_t& x, const control_vector_t& u, const SCALAR& t = 0.0) {
		x_ = x;
		u_ = u;
		t_ = t + t_shift_;
	}

	/**
	 * \brief sets current state, control and time
	 *
	 * Get the current state, control and time of the cost function. In this function, the user can add pre-computations
	 * shared between different calls to the derivative functions.
	 * @param x state vector
	 * @param u control vector
	 * @param t time
	 */
	virtual void getCurrentStateAndControl(Eigen::Matrix<SCALAR, STATE_DIM, 1> &x, Eigen::Matrix<SCALAR, CONTROL_DIM, 1> &u, SCALAR& t) const
	{
		x = this->x_;
		u = this->u_;
		t = this->t_;
	}

	/**
	 * \brief evaluate intermediate costs
	 *
	 * Evaluates the running/intermediate cost function for the control, state and time set in setCurrentStateAndControl()
	 * @return costs
	 */
	virtual SCALAR evaluateIntermediate() = 0;

	/**
	 * \brief evaluate terminal costs
	 *
	 * Evaluates the terminal cost for a given state and control set in setCurrentStateAndControl(). This usually ignores time.
	 * @return costs
	 */
	virtual SCALAR evaluateTerminal() = 0;

	virtual void shiftTime(const SCALAR t)
	{
		t_shift_ = t;
	}


protected:
	state_vector_t x_; /** state vector */
	control_vector_t u_; /** control vector */
	SCALAR t_; /** time */

	SCALAR t_shift_;
};

} // namespace optcon
} // namespace ct

#endif /* COSTFUNCTIONBASE_HPP_ */
