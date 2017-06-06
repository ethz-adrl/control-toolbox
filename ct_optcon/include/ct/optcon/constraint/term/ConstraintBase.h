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

#ifndef CT_OPTCON_CONSTRAINT_TERM_CONSTRAINTBASE_H_
#define CT_OPTCON_CONSTRAINT_TERM_CONSTRAINTBASE_H_

#include <ct/core/core.h>
#include <Eigen/Sparse>
#include <ct/core/internal/traits/TraitSelector.h>
#include <ct/core/internal/traits/TraitSelectorSpecs.h>

namespace ct {
namespace optcon {
namespace tpl {

/**
 * @ingroup    Constraint
 *
 * @brief      Base class for the constraints used in this toolbox
 *
 * @tparam     STATE_DIM  The state dimension
 * @tparam     INPUT_DIM  The control dimension
 * @tparam     SCALAR     The Scalar type
 */
template <size_t STATE_DIM, size_t INPUT_DIM, typename SCALAR>
class ConstraintBase {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	typedef typename ct::core::tpl::TraitSelector<SCALAR>::Trait Trait;
	typedef Eigen::Matrix<int, Eigen::Dynamic, 1> VectorXi;
	typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> VectorXs;
	typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic> MatrixXs;

	/**
	 * @brief      Custom constructor
	 *
	 * @param[in]  name  The name of the constraint
	 */
	ConstraintBase(std::string name = "Unnamed") :
		tAd_(SCALAR(0)),
		xAd_(core::StateVector<STATE_DIM, SCALAR>::Zero()),
		uAd_(core::ControlVector<INPUT_DIM, SCALAR>::Zero()),
		t_(0),
		x_(core::StateVector<STATE_DIM>::Zero()),
		u_(core::ControlVector<INPUT_DIM>::Zero()),
		name_(name)
		{

		}

	/**
	 * @brief      Copy constructor
	 *
	 * @param[in]  arg   The object to be copied
	 */
	ConstraintBase(const ConstraintBase& arg):
		tAd_(arg.tAd_),
		xAd_(arg.xAd_),
		uAd_(arg.uAd_),
		t_(arg.t_),
		x_(arg.x_),
		u_(arg.u_),
		lb_(arg.lb_),
		ub_(arg.ub_),
		name_(arg.name_)
		{}

	/**
	 * @brief      Creates a new instance of the object with same properties than original.
	 *
	 * @return     Copy of this object.
	 */
	virtual ConstraintBase<STATE_DIM, INPUT_DIM, SCALAR>* clone () const = 0;

	/**
	 * @brief      Destructor
	 */
	virtual ~ConstraintBase() {}

	/**
	 * @brief      Sets the SCALAR typed state, control, time
	 *
	 * @param[in]  x     The state vector
	 * @param[in]  u     The control vector
	 * @param[in]  t     The time
	 */
	virtual void setTimeStateInputAd(
			const core::StateVector<STATE_DIM, SCALAR> &x,
			const core::ControlVector<INPUT_DIM, SCALAR> &u,
			const SCALAR t){
		xAd_ = x;
		uAd_ = u;
		tAd_ = t;
	}

	/**
	 * @brief      Sets the double typed state, control, time
	 *
	 * @param[in]  x     The state vector
	 * @param[in]  u     The control vector
	 * @param[in]  t     The time
	 */
	virtual void setTimeStateInputDouble(
			const core::StateVector<STATE_DIM> &x,
			const core::ControlVector<INPUT_DIM> &u,
			const double t
		)
	{
		x_ = x;
		u_ = u;
		t_ = t;
	}

	/**
	 * @brief      Returns the number of constraints
	 *
	 * @return     The number of constraints
	 */
	virtual size_t getConstraintsCount() = 0;

	/**
	 * @brief      The evaluation of the constraint violation. Note this method
	 *             is SCALAR typed
	 *
	 * @return     The constraint violation
	 */
	virtual VectorXs evaluate() = 0;

	/**
	 * @brief      Returns the constraint type (equality or inequality)
	 *
	 * @return     The constraint type.
	 */
	virtual int getConstraintType() = 0;

	/**
	 * @brief      Returns the constraint jacobian wrt state
	 *
	 * @return     The constraint jacobian
	 */
	virtual Eigen::MatrixXd JacobianState() 
	{ 
		throw std::runtime_error("This constraint function element is not implemented for the given term."
		"Please use either auto-diff cost function or implement the analytical derivatives manually."); 
	}

	/**
	 * @brief      Returns the constraint jacobian wrt input
	 *
	 * @return     The constraint jacobian
	 */
	virtual Eigen::MatrixXd JacobianInput() 
	{ 
		throw std::runtime_error("This constraint function element is not implemented for the given term." 
		"Please use either auto-diff cost function or implement the analytical derivatives manually."); 
	}

	/**
	 * @brief      Returns the lower constraint bound
	 *
	 * @return     The lower constraint bound
	 */
	virtual Eigen::VectorXd getLowerBound() const
	{
		return lb_;
	}

	/**
	 * @brief      Returns the upper constraint bound
	 *
	 * @return     The upper constraint bound
	 */
	virtual Eigen::VectorXd getUpperBound() const
	{
		return ub_;
	}

	/**
	 * @brief      Returns the constraint name
	 *
	 * @param[out]      constraintName  The constraint name
	 */
	void getName(std::string& constraintName) const { constraintName=name_; }

	/**
	 * @brief      Sets the constraint name.
	 *
	 * @param[in]  constraintName  The constraint name
	 */
	void setName(const std::string constraintName) { name_=constraintName; }

	/**
	 * @brief      Returns the number of nonzeros in the jacobian wrt state
	 *
	 * @return     The number of non zeros
	 */
	virtual size_t getNumNonZerosJacobianState()
	{
		return STATE_DIM * getConstraintsCount();
	}

	/**
	 * @brief      Returns the number of nonzeros in the jacobian wrt control input
	 *
	 * @return     The number of non zeros
	 */
	virtual size_t getNumNonZerosJacobianInput()
	{
		return INPUT_DIM * getConstraintsCount();
	}

	/**
	 * @brief      Returns the constraint jacobian wrt state in sparse structure
	 *
	 * @return     The sparse constraint jacobian
	 */
	virtual Eigen::VectorXd jacobianStateSparse()
	{
		Eigen::VectorXd jac(Eigen::Map<Eigen::VectorXd>(JacobianState().data(), JacobianState().rows() * JacobianState().cols()));
		return jac;
	}

	/**
	 * @brief      Returns the constraint jacobian wrt control input in sparse structure
	 *
	 * @return     The sparse constraint jacobian
	 */
	virtual Eigen::VectorXd jacobianInputSparse()
	{
		Eigen::VectorXd jac(Eigen::Map<Eigen::VectorXd>(JacobianInput().data(), JacobianInput().rows() * JacobianInput().cols()));
		return jac;
	}


	/**
	 * @brief      Generates the sparsity pattern of the jacobian wrt state
	 *
	 * @param      rows  The vector of the row indices containing non zero
	 *                   elements in the constraint jacobian
	 * @param      cols  The vector of the column indices containing non zero
	 *                   elements in the constraint jacobian
	 */
	virtual void sparsityPatternState(VectorXi& rows, VectorXi& cols)
	{
		genBlockIndices(getConstraintsCount(), STATE_DIM, rows, cols);

	}

	/**
	 * @brief      Generates the sparsity pattern of the jacobian wrt control input
	 *
	 * @param      rows  The vector of the row indices containing non zero
	 *                   elements in the constraint jacobian
	 * @param      cols  The vector of the column indices containing non zero
	 *                   elements in the constraint jacobian
	 */
	virtual void sparsityPatternInput(VectorXi& rows, VectorXi& cols)
	{
		genBlockIndices(getConstraintsCount(), INPUT_DIM, rows, cols);	
	}


protected:

	SCALAR tAd_;
	core::StateVector<STATE_DIM, SCALAR> xAd_;
	core::ControlVector<INPUT_DIM, SCALAR> uAd_;
	double t_;
	core::StateVector<STATE_DIM> x_;
	core::ControlVector<INPUT_DIM> u_;

	Eigen::VectorXd lb_; // lower bound on the constraints
	Eigen::VectorXd ub_; // upper bound on the constraints

	/**
	 * @brief      Generates indices of a diagonal square matrix
	 *
	 * @param[in]  num_elements  The number of elements
	 * @param[out] iRow_vec      The row vector
	 * @param[out] jCol_vec      The column vector
	 */
	void genDiagonalIndices(
			const size_t num_elements,
			Eigen::VectorXi& iRow_vec,
			Eigen::VectorXi& jCol_vec)
	{
		iRow_vec.resize(num_elements);
		jCol_vec.resize(num_elements);

		size_t count = 0;

		for(size_t i = 0; i < num_elements; ++i){
			iRow_vec(count) = i;
			jCol_vec(count) = i;
			count++;
		}	
	}

	/**
	 * @brief      Generates indices of a full matrix
	 *
	 * @param[in]  num_rows  The number of rows of the matrix
	 * @param[in]  num_cols  The number columns of the matrix
	 * @param[out] iRow_vec  The row vector
	 * @param[out] jCol_vec  The col vector
	 */
	void genBlockIndices(
			const size_t num_rows,
			const size_t num_cols,
			Eigen::VectorXi& iRow_vec,
			Eigen::VectorXi& jCol_vec)
	{
		size_t num_gen_indices = num_rows*num_cols;

		iRow_vec.resize(num_gen_indices);
		jCol_vec.resize(num_gen_indices);

		size_t count = 0;

		for(size_t row = 0; row <num_rows; ++row){
			for(size_t col = 0; col < num_cols; ++col){
				iRow_vec(count) = row;
				jCol_vec(count) = col;
				count++;
			}
		}
	}


private:
	std::string name_;

};

} // namespace tpl

template<size_t STATE_DIM, size_t INPUT_DIM>
using ConstraintBase = tpl::ConstraintBase<STATE_DIM, INPUT_DIM, double>;

} // namespace optcon
} // namespace ct

#endif
