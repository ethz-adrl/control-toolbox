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

#ifndef CT_OPTCON_CONSTRAINTS_LINEARCONSTRAINTBASE_H_
#define CT_OPTCON_CONSTRAINTS_LINEARCONSTRAINTBASE_H_

#include "ConstraintContainerBase.h"

namespace ct {
namespace optcon {


/**
 * @ingroup    Constraint
 *
 * @brief      A base function for linear constraint functions which have a
 *             first derivative
 *
 * * The LinearConstraintBase Class is the base class for defining the
 *   non-linear optimization constraints.
 *
 * @tparam     STATE_DIM  Dimension of the state vector
 * @tparam     INPUT_DIM  Dimension of the control input vector
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class LinearConstraintContainer : public ConstraintContainerBase<STATE_DIM, INPUT_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef core::StateVector<STATE_DIM> state_vector_t;
	typedef core::ControlVector<INPUT_DIM> input_vector_t;

	typedef LinearConstraintContainer<STATE_DIM, INPUT_DIM>* LinearConstraintContainer_Raw_Ptr_t;
	typedef std::shared_ptr<LinearConstraintContainer<STATE_DIM, INPUT_DIM>> LinearConstraintContainer_Shared_Ptr_t;

	/**
	 * @brief      Default constructor
	 */
	LinearConstraintContainer() {}


	/**
	 * @brief      Copy constructor
	 *
	 * @param[in]  arg   The object to be copied
	 */
	LinearConstraintContainer(const LinearConstraintContainer& arg)
	{}

	/**
	 * @brief      Destructor
	 *
	 */
	virtual ~LinearConstraintContainer() {}

	/**
	 * Clones the linear constraint class
	 * @return pointer to the clone
	 */
	virtual LinearConstraintContainer_Raw_Ptr_t clone() const = 0;

	/**
	 * @brief      Overrides the base class implementation
	 *
	 * @param[in]  t     The time
	 * @param[in]  x     The state vector
	 * @param[in]  u     The control input vector
	 */
	virtual void setTimeStateInput(const double t, const state_vector_t& x, const input_vector_t& u) = 0;

	virtual void evaluate(Eigen::VectorXd& g, size_t& count) = 0;

	virtual void getLowerBound(Eigen::VectorXd& lb, size_t& count) = 0;

	virtual void getUpperBound(Eigen::VectorXd& ub, size_t& count) = 0;

	/**
	 * @brief      Evaluates the jacobian in sparse format
	 *
	 * @param[out]      jacVec  The vector of the non zero entries in the jacobian
	 * @param[out]      count   The size of jacVec
	 */
	virtual void evalJacSparse(Eigen::VectorXd& jacVec, size_t& count) = 0;

	/**
	 * @brief      Returns the full jacobian
	 *
	 * @return     The full jacobian
	 */
	virtual Eigen::MatrixXd evalJacDense() = 0;

	/**
	 * @brief      Evaluates the constraint jacobian wrt the state using sparse representation
	 *
	 * @param      jacVec  The sparse jacobian vector
	 * @param      count   The size of jacVec
	 */
	virtual void evalJacStateSparse(Eigen::VectorXd& jacVec, size_t& count) = 0;

	/**
	 * @brief      Evaluates the constraint jacobian wrt the state
	 *
	 * @return     The jacobian wrt state
	 */
	virtual Eigen::MatrixXd evalJacStateDense() = 0;

	/**
	 * @brief      Evaluates the constraint jacobian wrt the control input using sparse representation
	 *
	 * @param      jacVec  The sparse jacobian vector
	 * @param      count   The size of jacVec
	 */
	virtual void evalJacInputSparse(Eigen::VectorXd& jacVec, size_t& count) = 0;

	/**
	 * @brief      Evaluates the constraint jacobian wrt the control input
	 *
	 * @return     The jacobian wrt control
	 */
	virtual Eigen::MatrixXd evalJacInputDense() = 0;

	/**
	 * @brief      Returns the sparsity pattern for the full jacobian
	 *
	 * @param      iRows  The vector of the row indices containing non zero
	 *                    elements in the constraint jacobian
	 * @param      jCols  The vector of the column indices containing non zero
	 *                    elements in the constraint jacobian
	 *
	 * @return     The size of iRow and jCols
	 */
	virtual size_t generateSparsityPatternJacobian(Eigen::VectorXi& iRows, Eigen::VectorXi& jCols) = 0;

	/**
	 * @brief      Returns the sparsity pattern for the jacobian wrt state
	 *
	 * @param      iRows  The vector of the row indices containing non zero
	 *                    elements in the constraint jacobian
	 * @param      jCols  The vector of the column indices containing non zero
	 *                    elements in the constraint jacobian
	 *
	 * @return     The size of iRow and jCols
	 */
	virtual size_t generateSparsityPatternJacobianState(Eigen::VectorXi& iRows, Eigen::VectorXi& jCols) = 0;

	/**
	 * @brief      Returns the sparsity pattern for the jacobian wrt control
	 *
	 * @param      iRows  The vector of the row indices containing non zero
	 *                    elements in the constraint jacobian
	 * @param      jCols  The vector of the column indices containing non zero
	 *                    elements in the constraint jacobian
	 *
	 * @return     The size of iRow and jCols
	 */
	virtual size_t generateSparsityPatternJacobianInput(Eigen::VectorXi& iRows, Eigen::VectorXi& jCols) = 0;

	/**
	 * @brief      Returns the number of non zero elements in the full constraint jacobian
	 *
	 * @return     The number of the non zeros
	 */
	virtual size_t getConstraintJacobianNonZeroCount() = 0;

	/**
	 * @brief      Returns the number of non zero elements in the constraint jacobian wrt state
	 *
	 * @return     The number of the non zeros
	 */
	virtual size_t getConstraintJacobianStateNonZeroCount() = 0;

	/**
	 * @brief      Returns the number of non zero elements in the constraint jacobian wrt input
	 *
	 * @return     The number of the non zeros
	 */	
	virtual size_t getConstraintJacobianInputNonZeroCount() = 0;

	/**
	 * @brief      Returns the number of constraints
	 *
	 * @return     The number of constraints
	 */
	virtual size_t getConstraintCount() = 0;

	/**
	 * @brief      Returns the constraint types
	 *
	 * @param      constraint_types  The constraint types
	 */
	virtual void getConstraintTypes(Eigen::VectorXd& constraint_types) = 0;


protected:
	/**
	 * This is called by setTimeStateInput() method class. It can be used for updating the class members with the new state and input.
	 */
	virtual void update() = 0;
};

} // namespace optcon
} // namespace ct

#endif /* CT_OPTCON_LINEARCONSTRAINTBASE_H_ */
