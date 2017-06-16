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

#ifndef CT_OPTCON_CONSTRAINTS_EMPTY_LINEARCONSTRAINT_CONTAINER_H_
#define CT_OPTCON_CONSTRAINTS_EMPTY_LINEARCONSTRAINT_CONTAINER_H_

#include "LinearConstraintContainer.h"

namespace ct {
namespace optcon {


/**
 * @ingroup    Constraint
 *
 * @brief      simply an empty constraint
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class EmptyLinearConstraintContainer : public LinearConstraintContainer<STATE_DIM, INPUT_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef core::StateVector<STATE_DIM> state_vector_t;
	typedef core::ControlVector<INPUT_DIM> input_vector_t;

	typedef EmptyLinearConstraintContainer<STATE_DIM, INPUT_DIM>* EmptyLinearConstraintContainer_Raw_Ptr_t;
	typedef std::shared_ptr<EmptyLinearConstraintContainer<STATE_DIM, INPUT_DIM>> EmptyLinearConstraintContainer_Shared_Ptr_t;

	EmptyLinearConstraintContainer() {}

	EmptyLinearConstraintContainer(const EmptyLinearConstraintContainer& arg)
	{}

	virtual ~EmptyLinearConstraintContainer() {}

	virtual EmptyLinearConstraintContainer_Raw_Ptr_t clone() const override {return new EmptyLinearConstraintContainer();}

	virtual void setTimeStateInput(const double t, const state_vector_t& x, const input_vector_t& u) override {};

	virtual void evaluate(Eigen::VectorXd& g, size_t& count) override {};

	virtual void getLowerBound(Eigen::VectorXd& lb, size_t& count) override {};

	virtual void getUpperBound(Eigen::VectorXd& ub, size_t& count) override {};

	virtual void evalJacSparse(Eigen::VectorXd& jacVec, size_t& count) override {};

	virtual Eigen::MatrixXd evalJacDense() override {return Eigen::MatrixXd();};

	virtual void evalJacStateSparse(Eigen::VectorXd& jacVec, size_t& count) override {}

	virtual Eigen::MatrixXd evalJacStateDense() override {return Eigen::MatrixXd();}

	virtual void evalJacInputSparse(Eigen::VectorXd& jacVec, size_t& count) override {}

	virtual Eigen::MatrixXd evalJacInputDense() override {return Eigen::MatrixXd();}

	virtual size_t generateSparsityPatternJacobian(Eigen::VectorXi& iRows, Eigen::VectorXi& jCols) override {return 0;};

	virtual size_t generateSparsityPatternJacobianState(Eigen::VectorXi& iRows, Eigen::VectorXi& jCols) override {return 0;}

	virtual size_t generateSparsityPatternJacobianInput(Eigen::VectorXi& iRows, Eigen::VectorXi& jCols) override {return 0;}

	virtual size_t getConstraintJacobianNonZeroCount() override {return 0;}

	virtual size_t getConstraintJacobianStateNonZeroCount() override {return 0;}

	virtual size_t getConstraintJacobianInputNonZeroCount() override {return 0;}

	virtual size_t getConstraintCount() override {return 0;}

	virtual void getConstraintTypes(Eigen::VectorXd& constraint_types) override {}

protected:

	virtual void update() override {};
};

} // namespace optcon
} // namespace ct

#endif /* CT_OPTCON_LINEARCONSTRAINTBASE_H_ */
