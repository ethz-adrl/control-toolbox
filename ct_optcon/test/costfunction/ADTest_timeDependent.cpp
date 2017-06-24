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

#define EIGEN_INITIALIZE_MATRICES_BY_NAN

#include <ct/optcon/costfunction/CostFunctionAD.hpp>
#include <ct/optcon/costfunction/CostFunctionAnalytical.hpp>
#include <ct/optcon/costfunction/term/TermBase.hpp>
#include <ct/optcon/costfunction/term/TermLinear.hpp>
#include <ct/optcon/costfunction/term/TermQuadratic.hpp>
#include <ct/optcon/costfunction/term/TermOther.hpp>

#include <gtest/gtest.h>

namespace ct{
namespace optcon{
namespace example{

/*
 * Define a cost function term of form cost = c*t^2 (x'Qx + u'Ru)
 * analytical derivatives:
 * state Derivative: 	2*c*t^2 Qx
 * control Derivative: 	2*c*t^2 Rx
 * state second Derivative: 	2*c*t^2 Q
 * control second Derivative: 	2*c*t^2 R
 * */

template <size_t STATE_DIM, size_t CONTROL_DIM, typename S = double>
class TestTerm : public TermBase<STATE_DIM, CONTROL_DIM, S, double> {

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef S SCALAR;

	CT_OPTCON_DEFINE_TERM_TYPES

	TestTerm();

	TestTerm(const state_matrix_t& Q, const control_matrix_t& R, const S& c):
		Q_(Q),
		R_(R),
		c_(c)
	{
		x_ref_.setZero();	// default values
		u_ref_.setZero();
	}

	TestTerm(const TestTerm& arg):
		TermBase<STATE_DIM, CONTROL_DIM, S, double>(arg),
		Q_(arg.Q_),
		R_(arg.R_),
		c_(arg.c_),
		x_ref_(arg.x_ref_),
		u_ref_(arg.u_ref_)
		{}

	~TestTerm(){}

	TestTerm<STATE_DIM, CONTROL_DIM, S>* clone () const override {
		return new TestTerm(*this);
	}

	void setWeights(const state_matrix_t& Q, const control_matrix_t& R, const double& c){
		Q_ = Q;
		R_ = R;
		c_ = c;
	}

	void setStateAndControlReference(const core::StateVector<STATE_DIM, S>& x_ref, core::ControlVector<CONTROL_DIM, S>& u_ref){
		x_ref_ = x_ref;
		u_ref_ = u_ref;
	}

	S evaluate(const Eigen::Matrix<S, STATE_DIM, 1> &x, const Eigen::Matrix<S, CONTROL_DIM, 1> &u, const SCALAR& t) override{

		Eigen::Matrix<S, STATE_DIM, 1> xDiff = (x-x_ref_.template cast<S>());
		Eigen::Matrix<S, CONTROL_DIM, 1> uDiff = (u-u_ref_.template cast<S>());

		return ((xDiff.transpose() * Q_.template cast<S>() * xDiff) *  c_* t * t + (uDiff.transpose() * R_.template cast<S>() * uDiff)* c_* t * t)(0,0);
	}


	// todo: this is quite ugly
	core::StateVector<STATE_DIM, S> stateDerivativeAnalytical(const core::StateVector<STATE_DIM>& x, const core::ControlVector<CONTROL_DIM>&u, const double t) {
		return  (2 * c_* (S)t *(S)t * Q_* x. template cast<S>());
	}

	// todo: this is very ugly
	core::ControlVector<CONTROL_DIM, S> controlDerivativeAnalytical(const core::StateVector<STATE_DIM>& x, const core::ControlVector<CONTROL_DIM>&u, double t){
		return  (2 * c_* (S)t *(S)t * R_* u. template cast<S>());
	}

	// todo: this is quite ugly
	state_matrix_t stateSecondDerivativeAnalytical(const core::StateVector<STATE_DIM>& x, const core::ControlVector<CONTROL_DIM>&u, const double t) {
		return  (2 * c_* (S)t *(S)t * Q_);
	}

	// todo: this is very ugly
	control_matrix_t controlSecondDerivativeAnalytical(const core::StateVector<STATE_DIM>& x, const core::ControlVector<CONTROL_DIM>&u, double t)
	{
		return  (2 * c_* (S)t *(S)t * R_);
	}


protected:
	state_matrix_t Q_;
	control_matrix_t R_;
	S c_;

	core::StateVector<STATE_DIM, S> x_ref_;
	core::ControlVector<CONTROL_DIM, S> u_ref_;

};



TEST(AD_TEST_TIME_VAR, AD_TEST_TIME_VAR)
{
	using namespace ct;

	typedef CppAD::AD<double> SCALAR;

	Eigen::Matrix<SCALAR, 3, 3> Q; Q.setIdentity();
	Eigen::Matrix<SCALAR, 3, 3> R; R.setIdentity();

	Eigen::Matrix<SCALAR, 3, 3> Q_f = 10 * Q;
	Eigen::Matrix<SCALAR, 3, 3> R_f = 10 * R;

	SCALAR c = 1.0;
	SCALAR c_f = 2.0;

	std::shared_ptr<TestTerm<3, 3, SCALAR> > term_intermediate (new TestTerm<3, 3, SCALAR>(Q, R, c));
	std::shared_ptr<TestTerm<3, 3, SCALAR> > term_final (new TestTerm<3, 3, SCALAR>(Q_f, R_f, c_f));

	// autodiff costfunction
	std::shared_ptr<ct::optcon::CostFunctionAD<3, 3>> ADcf (new ct::optcon::CostFunctionAD<3, 3>());
	ADcf->addIntermediateTerm(term_intermediate);
	ADcf->addFinalTerm(term_final);


	Eigen::Vector3d x;	x.setRandom();
	Eigen::Vector3d u;	u.setRandom();

	double t_final = 4.0;

	for(double t = 0.0; t<=t_final; t=t+1){
		ADcf->setCurrentStateAndControl(x, u, t);

		Eigen::Matrix<SCALAR, 3, 1> diff1 = (ADcf->stateDerivativeTerminal()).template cast<SCALAR>() -  (term_final->stateDerivativeAnalytical(x, u, t));
		ASSERT_TRUE(diff1.maxCoeff() - diff1.minCoeff() < 1e-9);

		Eigen::Matrix<SCALAR, 3, 1> diff2 = (ADcf->stateDerivativeIntermediate()).template cast<SCALAR>() -  (term_intermediate->stateDerivativeAnalytical(x, u, t));
		ASSERT_TRUE(diff2.maxCoeff() - diff2.minCoeff() < 1e-9);

		Eigen::Matrix<SCALAR, 3, 1> diff3 = (ADcf->controlDerivativeIntermediate()).template cast<SCALAR>() -  (term_intermediate->controlDerivativeAnalytical(x, u, t));
		ASSERT_TRUE(diff3.maxCoeff() - diff3.minCoeff() < 1e-9);

		Eigen::Matrix<SCALAR, 3, 1> diff4 = (ADcf->controlDerivativeTerminal()).template cast<SCALAR>() -  (term_final->controlDerivativeAnalytical(x, u, t));
		ASSERT_TRUE(diff4.maxCoeff() - diff4.minCoeff() < 1e-9);

		Eigen::Matrix<SCALAR, 3, 3> diff5 = (ADcf->controlSecondDerivativeIntermediate()).template cast<SCALAR>() -  (term_intermediate->controlSecondDerivativeAnalytical(x, u, t));
		ASSERT_TRUE(diff5.maxCoeff() - diff5.minCoeff() < 1e-9);

		Eigen::Matrix<SCALAR, 3, 3> diff6 = (ADcf->controlSecondDerivativeTerminal()).template cast<SCALAR>() -  (term_final->controlSecondDerivativeAnalytical(x, u, t));
		ASSERT_TRUE(diff6.maxCoeff() - diff6.minCoeff() < 1e-9);

		Eigen::Matrix<SCALAR, 3, 3> diff7 = (ADcf->stateSecondDerivativeIntermediate()).template cast<SCALAR>() -  (term_intermediate->stateSecondDerivativeAnalytical(x, u, t));
		ASSERT_TRUE(diff7.maxCoeff() - diff7.minCoeff() < 1e-9);

		Eigen::Matrix<SCALAR, 3, 3> diff8 = (ADcf->stateSecondDerivativeTerminal()).template cast<SCALAR>() -  (term_final->stateSecondDerivativeAnalytical(x, u, t));
		ASSERT_TRUE(diff8.maxCoeff() - diff8.minCoeff() < 1e-9);
	}
}

} // namespace example
} // namespace optcon
} // namespace ct

/*!
 * \warning This test illustrates that time-dependency is not yet accounted for when using CppAD!
 */
int main(int argc, char **argv)
{
	using namespace ct::optcon::example;
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}


