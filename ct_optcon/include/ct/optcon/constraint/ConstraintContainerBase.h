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

#ifndef CT_OPTCON_CONSTRAINTCONTAINERBASE_H_
#define CT_OPTCON_CONSTRAINTCONTAINERBASE_H_

#include <ct/core/types/StateVector.h>
#include <ct/core/types/ControlVector.h>

namespace ct {
namespace optcon {

/** \defgroup Constraint Constraint
 *
 * \brief Constraint Module used in Optimal Control
 */

/**
 * @ingroup    Constraint
 * + { list_item_description }
 *
 * @brief      The ConstraintBase Class is the base class for defining the
 *             non-linear optimization constraints.
 *
 * @tparam     STATE_DIM  Dimension of the state vector
 * @tparam     CONTROL_DIM  Dimension of the control input vector
 *
 *             An example for usage is given in the unit test @ref
 *             ConstraintTest.h
 */

template <size_t STATE_DIM, size_t CONTROL_DIM>
class ConstraintContainerBase
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef core::StateVector<STATE_DIM> state_vector_t;
	typedef core::ControlVector<CONTROL_DIM> input_vector_t;

	typedef ConstraintContainerBase<STATE_DIM, CONTROL_DIM>* ConstraintBase_Raw_Ptr_t;

	/**
	 * \brief Default constructor
	 *
	 * Default constructor, sets state, control and time to zero
	 */
	ConstraintContainerBase() :
		x_(state_vector_t::Zero()),
		u_(input_vector_t::Zero()),
		t_(0.0)
	{}

	/**
	 * \brief copy constructor
	 *
	 * Copy constructor
	 */
	ConstraintContainerBase(const ConstraintContainerBase& arg) :
		x_(arg.x_),
		u_(arg.u_),
		t_(arg.t_),
		lowerBoundsIntermediate_(arg.lowerBoundsIntermediate_),
		lowerBoundsTerminal_(arg.lowerBoundsTerminal_),
		upperBoundsIntermediate_(arg.upperBoundsIntermediate_),
		upperBoundsTerminal_(arg.upperBoundsTerminal_)
	{}

	/**
	 * \brief Destructor
	 *
	 * Destructor
	 */
	virtual ~ConstraintContainerBase() {}

	/**
	 * Clones the constraint.
	 * @return Base pointer to the clone
	 */
	virtual ConstraintBase_Raw_Ptr_t clone() const = 0;

	/**
	 * This methods updates the current time, state, and input in the class. It also call the user defined update() method,
	 * which can be used to make the implementation more efficient by executing shared calculations directly at the time of
	 * updating time, state and control.
	 *
	 * @param[in] t current time
	 * @param[in] x state vector
	 * @param[in] x input vector
	 */
	virtual void setCurrentStateAndControl(const state_vector_t& x,	const input_vector_t& u, const double t = 0.0)
	{
		t_ = t;
		x_ = x;
		u_ = u;
		update();
	}

	/**
	 * @brief      Evaluates the intermediate constraints
	 *
	 * @return     The evaluation of the intermediate constraints
	 */
	virtual Eigen::VectorXd evaluateIntermediate() = 0;

	/**
	 * @brief      Evaluates the terminal constraints
	 *
	 * @return     The evaluation of the terminal constraints
	 */
	virtual Eigen::VectorXd evaluateTerminal() = 0;

	/**
	 * @brief      Retrieves the number of intermediate constraints
	 *
	 * @return     The number of intermediate constraints
	 */
	virtual size_t getIntermediateConstraintsCount() = 0;

	/**
	 * @brief      Retrieves the number of final constraints
	 *
	 * @return     The number of final constraints
	 */
	virtual size_t getTerminalConstraintsCount() = 0;

	/**
	 * @brief      Retrieves the total number of constraints
	 *
	 * @return     The total number of constraints
	 */
	size_t getConstraintsCount()
	{
		return getIntermediateConstraintsCount() + getTerminalConstraintsCount();
	}

	/**
	 * @brief      Retrieves the lower constraint bound on the intermediate
	 *             constraints
	 *
	 * @return     The lower bound on the intermediate constraints
	 */
	Eigen::VectorXd getLowerBoundsIntermediate() const
	{
		return lowerBoundsIntermediate_;
	}

	/**
	 * @brief      Retrieves the lower constraint bound on the terminal
	 *             constraints
	 *
	 * @return     The lower bound on the terminal constraints
	 */
	Eigen::VectorXd getLowerBoundsTerminal() const
	{
		return lowerBoundsTerminal_;
	}

	/**
	 * @brief      Retrieves the upper constraint bound on the intermediate
	 *             constraints
	 *
	 * @return     The upper bound on the intermediate constraints
	 */
	Eigen::VectorXd getUpperBoundsIntermediate() const
	{
		return upperBoundsIntermediate_;
	}

	/**
	 * @brief      Retrieves the upper constraint bound on the terminal
	 *             constraints
	 *
	 * @return     The upper bound on the terminal constraints
	 */
	Eigen::VectorXd getUpperBoundsTerminal() const
	{	
		return upperBoundsTerminal_;
	}


protected:

	/**
	 * @brief      Gets called by the setCurrentStateAndControl method. Can be
	 *             used to update container properties
	 */
	virtual void update() = 0;

	state_vector_t   x_;	/** state vector */
	input_vector_t u_;		/** control vector */
	double t_;			    /** time */

	Eigen::VectorXd lowerBoundsIntermediate_;
	Eigen::VectorXd lowerBoundsTerminal_;
	Eigen::VectorXd upperBoundsIntermediate_;
	Eigen::VectorXd upperBoundsTerminal_;
};


} // namespace optcon
} // namespace ct

#endif /* CT_CONSTRAINTBASE_H_ */
