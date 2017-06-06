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
 * \ingroup Constraint
 * +
 *
 * \brief The ConstraintBase Class is the base class for defining the non-linear optimization constraints.
 *
 * @tparam STATE_DIM: Dimension of the state vector
 * @tparam INPUT_DIM: Dimension of the control input vector
 * @tparam CONSTRAINT2_DIM: Maximum number of pure state constraints
 * @tparam CONSTRAINT1_DIM: Maximum number of state-input constraints
 *
 * An example for usage is given in the unit test \ref ConstraintTest.cpp
 *
 */

template <size_t STATE_DIM, size_t INPUT_DIM>
class ConstraintContainerBase
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef core::StateVector<STATE_DIM> state_vector_t;
	typedef core::ControlVector<INPUT_DIM> input_vector_t;

	// typedef ConstraintDefinitions<STATE_DIM, INPUT_DIM, CONSTRAINT2_DIM, CONSTRAINT1_DIM> CONSTRAINT_DEFINITIONS;

	typedef ConstraintContainerBase<STATE_DIM, INPUT_DIM>* ConstraintBase_Raw_Ptr_t;
	typedef std::shared_ptr<ConstraintContainerBase<STATE_DIM, INPUT_DIM>> ConstraintBase_Shared_Ptr_t;


	/**
	 * \brief Default constructor
	 *
	 * Default constructor, sets state, control and time to zero
	 */
	ConstraintContainerBase() :
		t_(0.0)
	{
		x_.setZero();
		u_.setZero();
	}

	/**
	 * \brief copy constructor
	 *
	 * Copy constructor
	 */
	ConstraintContainerBase(const ConstraintContainerBase& arg) :
		t_(arg.t_),
		x_(arg.x_),
		u_(arg.u_),
		constraintLowerBounds_(arg.constraintLowerBounds_),
		constraintUpperBounds_(arg.constraintUpperBounds_)
	{}

	/**
	 * \brief Destructor
	 *
	 * Destructor
	 */
	virtual ~ConstraintContainerBase() {}

	/**
	 * This methods updates the current time, state, and input in the class. It also call the user defined update() method,
	 * which can be used to make the implementation more efficient by executing shared calculations directly at the time of
	 * updating time, state and control.
	 *
	 * @param[in] t current time
	 * @param[in] x state vector
	 * @param[in] x input vector
	 */
	virtual void setTimeStateInput(
			const double t,
			const state_vector_t& x,
			const input_vector_t& u)
	{
		t_ = t;
		x_ = x;
		u_ = u;
		update();
	}

	/**
	 * This method retrieves the state-input constraint vector
	 * @param[in] lowerBound The lower bound values for the state-input constraint
	 * @param[in] upperBound The upper bound values for the state-input constraint
	 */
	void setConstraintBounds(
			const Eigen::VectorXd& lowerBound,
			const Eigen::VectorXd& upperBound)
	{
		constraintLowerBounds_ = lowerBound;
		constraintUpperBounds_ = upperBound;
	}

	Eigen::VectorXd getLowerBounds() const
	{
		return constraintLowerBounds_;
	}

	Eigen::VectorXd getUpperBounds() const
	{
		return constraintUpperBounds_;
	}

	/**
	 * Clones the constraint.
	 * @return Base pointer to the clone
	 */
	virtual ConstraintBase_Raw_Ptr_t clone() const = 0;

	/**
	 * This method retrieves the pure state constraint vector
	 * @param[out] g2  The value of the pure state constraint
	 * @param[out] nc2 The number of the active pure state constraints
	 */
	virtual void evaluate(Eigen::VectorXd& g, size_t& n) = 0;

	/**
	 * This method retrieves the state-input constraint vector
	 * @param[out] g1  The value of the state-input constraint vector
	 * @param[out] nc1 The number of the active state-vector constraints
	 */

	virtual void getConstraintTypes(std::vector<Eigen::VectorXd>& constraint_types) {};


protected:

	/**
	 * This is called by setTimeStateInput() method class. It can be used for updating the class members with the new state and input.
	 * May help to make computations more efficient by executing shared calculations here.
	 */
	virtual void update() = 0;

	double t_;			    /** time */
	state_vector_t   x_;	/** state vector */
	input_vector_t u_;		/** control vector */

	Eigen::VectorXd constraintLowerBounds_;
	Eigen::VectorXd constraintUpperBounds_;
};


} // namespace optcon
} // namespace ct

#endif /* CT_CONSTRAINTBASE_H_ */
