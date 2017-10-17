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

#pragma once

#include <cppad/example/cppad_eigen.hpp>

#include "LinearConstraintContainer.h"
#include "term/ConstraintBase.h"


namespace ct {
namespace optcon {

/**
 * @ingroup    Constraint
 *
 * @brief      Contains all the constraints using analytically calculated
 *             jacobians
 *
 * @tparam     STATE_DIM  The state dimension
 * @tparam     CONTROL_DIM  The control dimension
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class ConstraintContainerAnalytical : public LinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef core::StateVector<STATE_DIM, SCALAR>   state_vector_t;
	typedef core::ControlVector<CONTROL_DIM, SCALAR> input_vector_t;

	typedef ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>* ConstraintContainerAnalytical_Raw_Ptr_t;
	typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> VectorXs;
	typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic> MatrixXs; 

	ConstraintContainerAnalytical();

	/**
	 * @brief      Constructor using state, control and time
	 *
	 * @param      x     state vector
	 * @param      u     control vector
	 * @param      t     time
	 */
	ConstraintContainerAnalytical(const state_vector_t &x, const input_vector_t &u, const SCALAR& t = 0.0);

	/**
	 * @brief      Copy constructor
	 *
	 * @param      arg   constraint class to copy
	 */
	ConstraintContainerAnalytical(const ConstraintContainerAnalytical& arg);

	/**
	 * @brief      Deep-cloning of Constraint
	 *
	 * @return     Copy of this object.
	 */
	virtual ConstraintContainerAnalytical_Raw_Ptr_t clone () const override;

	/**
	 * @brief      Destructor
	 */
	virtual ~ConstraintContainerAnalytical();

	/**
	 * @brief      Adds an intermedaite constraint.
	 *
	 * @param[in]  constraint  The constraint to be added
	 * @param[in]  verbose     Flag indicating whether verbosity is on or off
	 */
	void addIntermediateConstraint(std::shared_ptr<ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>> constraint, bool verbose);

	/**
	 * @brief      Adds a terminal constraint.
	 *
	 * @param[in]  constraint  The constraint to be added
	 * @param[in]  verbose     Flag indicating whether verbosity is on or off
	 */
	void addTerminalConstraint(std::shared_ptr<ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>> constraint, bool verbose);

	virtual VectorXs evaluateIntermediate() override;

	virtual VectorXs evaluateTerminal() override;

	virtual size_t getIntermediateConstraintsCount() override;

	virtual size_t getTerminalConstraintsCount() override;

	virtual VectorXs jacobianStateSparseIntermediate() override;

	virtual MatrixXs jacobianStateIntermediate() override;

	virtual VectorXs jacobianStateSparseTerminal() override;

	virtual MatrixXs jacobianStateTerminal() override;

	virtual VectorXs jacobianInputSparseIntermediate() override;

	virtual MatrixXs jacobianInputIntermediate() override;

	virtual VectorXs jacobianInputSparseTerminal() override;

	virtual MatrixXs jacobianInputTerminal() override;

	virtual void sparsityPatternStateIntermediate(Eigen::VectorXi& iRows, Eigen::VectorXi& jCols) override;

	virtual void sparsityPatternStateTerminal(Eigen::VectorXi& iRows, Eigen::VectorXi& jCols) override;

	virtual void sparsityPatternInputIntermediate(Eigen::VectorXi& iRows, Eigen::VectorXi& jCols) override;

	virtual void sparsityPatternInputTerminal(Eigen::VectorXi& iRows, Eigen::VectorXi& jCols) override;

	virtual size_t getJacobianStateNonZeroCountIntermediate() override;

	virtual size_t getJacobianStateNonZeroCountTerminal() override;

	virtual size_t getJacobianInputNonZeroCountIntermediate() override;

	virtual size_t getJacobianInputNonZeroCountTerminal() override;

	virtual bool initializeIntermediate() override;

	virtual bool initializeTerminal() override;


private:
	virtual void update() override;

	/**
	 * @brief      Checks whether the intermediate constraints are initialized.
	 *             Throws a runtime error if not.
	 */
	void checkIntermediateConstraints();

	/**
	 * @brief      Checks whether the terminal constraints are initialized.
	 *             Throws a runtime error if not.
	 */
	void checkTerminalConstraints();


	std::vector<std::shared_ptr<ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>>> constraintsIntermediate_;
	std::vector<std::shared_ptr<ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>>> constraintsTerminal_;

	VectorXs evalIntermediate_;
	VectorXs evalJacSparseStateIntermediate_;
	VectorXs evalJacSparseInputIntermediate_;
	MatrixXs evalJacDenseStateIntermediate_;
	MatrixXs evalJacDenseInputIntermediate_;

	VectorXs evalTerminal_;
	VectorXs evalJacSparseStateTerminal_;
	VectorXs evalJacSparseInputTerminal_;
	MatrixXs evalJacDenseStateTerminal_;
	MatrixXs evalJacDenseInputTerminal_;
};


}// namespace optcon
}// namespace ct

