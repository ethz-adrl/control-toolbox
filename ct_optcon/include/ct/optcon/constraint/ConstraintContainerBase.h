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

#ifndef CT_OPTCON_CONSTRAINT_CONSTRAINTCONTAINERBASE_H_
#define CT_OPTCON_CONSTRAINT_CONSTRAINTCONTAINERBASE_H_

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
 *
 * @brief      The ConstraintBase Class is the base class for defining the
 *             non-linear optimization constraints.
 *
 * @tparam     STATE_DIM  Dimension of the state vector
 * @tparam     INPUT_DIM  Dimension of the control input vector
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class ConstraintContainerBase
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef core::StateVector<STATE_DIM> state_vector_t;
	typedef core::ControlVector<INPUT_DIM> input_vector_t;

	typedef ConstraintContainerBase<STATE_DIM, INPUT_DIM>* ConstraintBase_Raw_Ptr_t;
	typedef std::shared_ptr<ConstraintContainerBase<STATE_DIM, INPUT_DIM>> ConstraintBase_Shared_Ptr_t;

	/**
	 * @brief      Default constructor, sets state, control and time to zero
	 *
	 */
	ConstraintContainerBase() :
		t_(0.0)
	{
		x_.setZero();
		u_.setZero();
	}

	/**
	 @brief      copy constructor
	
	 @param[in]  arg   The object to be copied
	*/
	ConstraintContainerBase(const ConstraintContainerBase& arg) :
		t_(arg.t_),
		x_(arg.x_),
		u_(arg.u_),
		constraintLowerBounds_(arg.constraintLowerBounds_),
		constraintUpperBounds_(arg.constraintUpperBounds_)
	{}


	/**
	 * @brief      Destructor
	 */
	virtual ~ConstraintContainerBase() {}

	/**
	 * This methods updates the current time, state, and input in the class. It
	 * also call the user defined update() method, which can be used to make the
	 * implementation more efficient by executing shared calculations directly
	 * at the time of updating time, state and control.
	 *
	 * @param[in]  t     current time
	 * @param[in]  x     state vector
	 * @param[in]  u     control vector
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
	 * @brief      This method retrieves the constraint bound
	 *
	 * @param[in]  lowerBound  The lower bound
	 * @param[in]  upperBound  The upper bound
	 */
	void setConstraintBounds(
			const Eigen::VectorXd& lowerBound,
			const Eigen::VectorXd& upperBound)
	{
		constraintLowerBounds_ = lowerBound;
		constraintUpperBounds_ = upperBound;
	}

	/**
	 * @brief      Returns the lower bound
	 *
	 * @return     The lower bounds.
	 */
	virtual Eigen::VectorXd getLowerBound() const
	{
		return constraintLowerBounds_;
	}

	/**
	 * @brief      Returns the upper bounds
	 *
	 * @return     The upper bounds.
	 */
	virtual Eigen::VectorXd getUpperBound() const
	{
		return constraintUpperBounds_;
	}

	/**
	 * @brief      Creates a new instance of the object with same properties than original.
	 *
	 * @return     Copy of this object.
	 */
	virtual ConstraintBase_Raw_Ptr_t clone() const = 0;

	
	/**
	 * @brief      Evaluates the constraint
	 *
	 * @param[out] g     The constraint violation
	 * @param[out] n     The size of the constraint
	 */
	virtual void evaluate(Eigen::VectorXd& g, size_t& n) = 0;

	/**
	 * @brief      This method returns the constraint type
	 *
	 * @param[out]      constraint_types  The constraint type
	 */
	virtual void getConstraintTypes(std::vector<Eigen::VectorXd>& constraint_types) {};


protected:

	/**
	 * @brief      Updates the constraints
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
