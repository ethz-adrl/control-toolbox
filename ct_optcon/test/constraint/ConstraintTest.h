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

#ifndef CT_OPTCON_CONSTRAINTTEST_HPP_
#define CT_OPTCON_CONSTRAINTTEST_HPP_


namespace ct{
namespace optcon{
namespace example{


/*!
 * This is an example for using constraints.
 * \example ConstraintTest.h
 */

const bool verbose = true;

const size_t state_dim = 10;
const size_t input_dim = 8;


//! A pure state constraint term
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
class PureStateConstraint_Example : public ct::optcon::tpl::ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	typedef typename ct::core::tpl::TraitSelector<SCALAR>::Trait Trait;
	typedef ct::optcon::tpl::ConstraintBase<state_dim, input_dim, SCALAR> Base;
	typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> VectorXs;

	PureStateConstraint_Example()
	{
		Base::lb_.resize(STATE_DIM);
		Base::ub_.resize(STATE_DIM);
		Base::lb_.setZero();
		Base::ub_.setZero();
	}

	PureStateConstraint_Example(const PureStateConstraint_Example& arg):
		Base(arg),
		A_(arg.A_) {}

	virtual ~PureStateConstraint_Example(){}

	virtual PureStateConstraint_Example<STATE_DIM, CONTROL_DIM, SCALAR>* clone () const override 
	{ 
		return new PureStateConstraint_Example(*this);
	}

	virtual size_t getConstraintSize() const override {return STATE_DIM;}

	virtual VectorXs evaluate(const Eigen::Matrix<SCALAR, STATE_DIM, 1> &x, const Eigen::Matrix<SCALAR, CONTROL_DIM, 1> &u, const SCALAR t) override
	 {
	 	return A_.template cast<SCALAR>() * x;
	 }

	virtual Eigen::MatrixXd jacobianState(const Eigen::Matrix<double, STATE_DIM, 1> &x, const Eigen::Matrix<double, CONTROL_DIM, 1> &u, const double t) override 
	{
		return A_;
	}

	void setA(const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& A){A_ = A;}

private:
	Eigen::Matrix<double, STATE_DIM, STATE_DIM> A_;
};

//! A state input constraint term
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
class StateInputConstraint_Example : public ct::optcon::tpl::ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	typedef typename ct::core::tpl::TraitSelector<SCALAR>::Trait Trait;
	typedef ct::optcon::tpl::ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR> Base;
	typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> VectorXs;

	StateInputConstraint_Example()
	{
		Base::lb_.resize(CONTROL_DIM);
		Base::ub_.resize(CONTROL_DIM);
		Base::lb_.setZero();
		Base::ub_.setZero();
	}

	StateInputConstraint_Example(const StateInputConstraint_Example& arg):
		ct::optcon::tpl::ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>(arg),
		A_(arg.A_),
		B_(arg.B_)
		{
			Base::lb_.resize(CONTROL_DIM);
			Base::ub_.resize(CONTROL_DIM);
			Base::lb_.setZero();
			Base::ub_.setZero();
		}

	virtual StateInputConstraint_Example<STATE_DIM, CONTROL_DIM, SCALAR>* clone () const override {return new StateInputConstraint_Example(*this);}

	virtual size_t getConstraintSize() const override {return CONTROL_DIM;}

	VectorXs evaluate(const Eigen::Matrix<SCALAR, STATE_DIM, 1> &x, const Eigen::Matrix<SCALAR, CONTROL_DIM, 1> &u, const SCALAR t) 
	{
		return (A_.template cast<SCALAR>() * x + B_.template cast<SCALAR>() * u);
	}

	virtual Eigen::MatrixXd jacobianState(const Eigen::Matrix<double, STATE_DIM, 1> &x, const Eigen::Matrix<double, CONTROL_DIM, 1> &u, const double t) override 
	{
		return A_;
	}

	virtual Eigen::MatrixXd jacobianInput(const Eigen::Matrix<double, STATE_DIM, 1> &x, const Eigen::Matrix<double, CONTROL_DIM, 1> &u, const double t) override 
	{
		return B_;
	}

	void setAB(const Eigen::Matrix<double, CONTROL_DIM, STATE_DIM>& A, const Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> B){
		A_ = A;
		B_ = B;
	}

private:
	Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> A_;
	Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> B_;
};


//! A simple example with an 1d constraint
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
class ConstraintTerm1D : public ct::optcon::tpl::ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	const static size_t term_dim = 1;
	typedef typename ct::core::tpl::TraitSelector<SCALAR>::Trait Trait;
	typedef ct::optcon::tpl::ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR> Base;
	typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> VectorXs;

	ConstraintTerm1D()
	{
		Base::lb_.resize(term_dim);
		Base::ub_.resize(term_dim);
		Base::lb_.setZero();
		Base::ub_.setZero();		
	}

	virtual ~ConstraintTerm1D(){}

	virtual ConstraintTerm1D<STATE_DIM, CONTROL_DIM, SCALAR>* clone () const override {return new ConstraintTerm1D<STATE_DIM, CONTROL_DIM, SCALAR>();}

	virtual size_t getConstraintSize() const override {return term_dim;}

	virtual VectorXs evaluate(const Eigen::Matrix<SCALAR, STATE_DIM, 1> &x, const Eigen::Matrix<SCALAR, CONTROL_DIM, 1> &u, const SCALAR t) override 
	{
		Eigen::Matrix<SCALAR, term_dim, 1>  constr_violation;
		constr_violation.template segment<1>(0) << (u(1)*Trait::cos(x(2)) - u(0)*Trait::sin(x(2)) - u(2));
		return constr_violation;
	}

	virtual Eigen::MatrixXd jacobianState(const Eigen::Matrix<double, STATE_DIM, 1> &x, const Eigen::Matrix<double, CONTROL_DIM, 1> &u, const double t) override{
		Eigen::Matrix<double, term_dim, STATE_DIM> jac; jac.setZero();
		jac << 0.0, 0.0, -u(1)*sin(x(2)) - u(0)*cos(x(2));
		return jac;
	}

	virtual Eigen::MatrixXd jacobianInput(const Eigen::Matrix<double, STATE_DIM, 1> &x, const Eigen::Matrix<double, CONTROL_DIM, 1> &u, const double t) override{
		Eigen::Matrix<double, term_dim, CONTROL_DIM> jac; jac.setZero();
		jac << -sin(x(2)), cos(x(2)), -1.0;
		return jac;
	}
};


//! A simple example with a 2d constraint
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
class ConstraintTerm2D : public ct::optcon::tpl::ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	const static size_t term_dim = 2;
	typedef typename ct::core::tpl::TraitSelector<SCALAR>::Trait Trait;
	typedef ct::optcon::tpl::ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR> Base;
	typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> VectorXs;

	ConstraintTerm2D()
	{
		Base::lb_.resize(term_dim);
		Base::ub_.resize(term_dim);
		Base::lb_.setZero();
		Base::ub_.setZero();		
	}

	virtual ~ConstraintTerm2D(){}

	ConstraintTerm2D<STATE_DIM, CONTROL_DIM, SCALAR>* clone () const override {return new ConstraintTerm2D<STATE_DIM, CONTROL_DIM, SCALAR>();}

	virtual size_t getConstraintSize() const override {return term_dim;}

	virtual VectorXs evaluate(const Eigen::Matrix<SCALAR, STATE_DIM, 1> &x, const Eigen::Matrix<SCALAR, CONTROL_DIM, 1> &u, const SCALAR t) override {
		Eigen::Matrix<SCALAR, term_dim, 1>  constr_violation;
		constr_violation(0) = (u(1)*Trait::cos(x(2)) - u(0)*Trait::sin(x(2)) - u(2));
		constr_violation(1) = (u(2)*Trait::cos(x(1)) - u(2)*Trait::sin(x(1)) - u(1));
		return constr_violation;
	}

	virtual Eigen::MatrixXd jacobianState(const Eigen::Matrix<double, STATE_DIM, 1> &x, const Eigen::Matrix<double, CONTROL_DIM, 1> &u, const double t) override {
		Eigen::Matrix<double, term_dim, STATE_DIM> jac; jac.setZero();
		jac.row(0) << 0.0, 0.0, -u(1)*sin(x(2)) - u(0)*cos(x(2));
		jac.row(1) << 0.0, -(u(2))*sin(x(1)) - u(2)*cos(x(1)), 0.0;

		return jac;
	}

	virtual Eigen::MatrixXd jacobianInput(const Eigen::Matrix<double, STATE_DIM, 1> &x, const Eigen::Matrix<double, CONTROL_DIM, 1> &u, const double t) override {
		Eigen::Matrix<double, term_dim, CONTROL_DIM> jac; jac.setZero();
		jac.row(0) << -sin(x(2)), cos(x(2)), -1.0;
		jac.row(1) << 0.0, -1.0,  cos(x(1)) - sin(x(1));
		return jac;
	}
};


TEST(pureStateConstraintTest, pureStateConstraintTest)
{
	typedef CppAD::AD<CppAD::cg::CG<double> > ScalarCg;
	std::shared_ptr<ct::optcon::ConstraintContainerAD<state_dim, input_dim>> constraintAD (
		new ct::optcon::ConstraintContainerAD<state_dim, input_dim>());

	std::shared_ptr<ct::optcon::ConstraintContainerAnalytical<state_dim, input_dim>> constraintAN (
		new ct::optcon::ConstraintContainerAnalytical<state_dim, input_dim>());

	Eigen::Matrix<double, state_dim, state_dim> A; A.setRandom();

	std::shared_ptr<PureStateConstraint_Example<state_dim, input_dim, ScalarCg>> term1_ad ( 
		new PureStateConstraint_Example<state_dim, input_dim, ScalarCg>());
	term1_ad->setName("term1_ad");
	term1_ad->setA(A);

	std::shared_ptr<PureStateConstraint_Example<state_dim, input_dim, double>> term1_an(
		 new PureStateConstraint_Example<state_dim, input_dim, double>());
	term1_an->setName("term1_an");
	term1_an->setA(A);

	constraintAD->addIntermediateConstraint(term1_ad, verbose);
	constraintAD->initialize();
	constraintAN->addIntermediateConstraint(term1_an, verbose);
	constraintAN->initialize();

	std::shared_ptr<ct::optcon::ConstraintContainerAnalytical<state_dim, input_dim>> constraintAN_cloned (constraintAN->clone());
	std::shared_ptr<ct::optcon::ConstraintContainerAD<state_dim, input_dim>> constraintAD_cloned (constraintAD->clone());

	size_t nRuns = 100;

	for(size_t i = 0; i<nRuns; i++){

		/* evaluate constraint */
		Eigen::VectorXd g1_ad, g1_an, g1_ad_cl, g1_an_cl;
		size_t g1_dim_ad, g1_dim_an, g1_dim_ad_cl, g1_dim_an_cl;

		Eigen::Matrix<double, state_dim, 1> state; state.setRandom();
		Eigen::Matrix<double, input_dim, 1> input; input.setRandom();

		std::random_device rd;
		std::mt19937 gen(rd());
		std::uniform_real_distribution<> time_distr(0, 100);
		double time = time_distr(gen);

		constraintAN->setCurrentStateAndControl(state, input, time);
		constraintAD->setCurrentStateAndControl(state, input, time);
		constraintAD_cloned->setCurrentStateAndControl(state, input, time);
		constraintAN_cloned->setCurrentStateAndControl(state, input, time);

		g1_an = constraintAN->evaluateIntermediate();
		g1_ad = constraintAD->evaluateIntermediate();
		g1_an_cl = constraintAN_cloned->evaluateIntermediate();
		g1_ad_cl = constraintAD_cloned->evaluateIntermediate();

		// test if constraint violations are the same
		ASSERT_TRUE(g1_an.isApprox(g1_ad));
		ASSERT_TRUE(g1_an.isApprox(g1_ad_cl));
		ASSERT_TRUE(g1_an.isApprox(g1_an_cl));

		// test if number of constraints are the same
		ASSERT_TRUE(g1_dim_an == g1_dim_ad);
		ASSERT_TRUE(g1_dim_an == g1_dim_an_cl);
		ASSERT_TRUE(g1_dim_an == g1_dim_ad_cl);

		Eigen::MatrixXd F_an, F_ad, F_cloned, F_cloned_an;
		F_an.setZero();
		F_ad.setZero();
		F_cloned.setZero();
		F_cloned_an.setZero();
		size_t count = 0;

		F_an = constraintAN->jacobianStateIntermediate();
		F_ad = constraintAD->jacobianStateIntermediate();
		F_cloned_an = constraintAN_cloned->jacobianStateIntermediate();
		F_cloned = constraintAD_cloned->jacobianStateIntermediate();

		// compare jacobians
			
		ASSERT_TRUE(F_an.isApprox(F_ad));
		ASSERT_TRUE(F_an.isApprox(F_cloned));
		ASSERT_TRUE(F_an.isApprox(F_cloned_an));
	}
}


TEST(stateInputConstraintTest, stateInputConstraintTest)
{
	typedef CppAD::AD<CppAD::cg::CG<double> > ScalarCg;
	std::shared_ptr<ct::optcon::ConstraintContainerAD<state_dim, input_dim>> constraintAD (
		new ct::optcon::ConstraintContainerAD<state_dim, input_dim>());
	std::shared_ptr<ct::optcon::ConstraintContainerAnalytical<state_dim, input_dim>> constraintAN (
		new ct::optcon::ConstraintContainerAnalytical<state_dim, input_dim>());

	Eigen::Matrix<double, input_dim, state_dim> A; A.setRandom();
	Eigen::Matrix<double, input_dim, input_dim> B; B.setRandom();

	std::shared_ptr<StateInputConstraint_Example<state_dim, input_dim, ScalarCg>> term1_ad ( 
		new StateInputConstraint_Example<state_dim, input_dim, ScalarCg>());
	term1_ad->setName("term1_ad");
	term1_ad->setAB(A, B);

	std::shared_ptr<StateInputConstraint_Example<state_dim, input_dim, double>> term1_an ( 
		new StateInputConstraint_Example<state_dim, input_dim, double>());
	term1_an->setName("term1_an");
	term1_an->setAB(A, B);


	// std::cout << "Adding terms to constraintAD" << std::endl;
	constraintAD->addIntermediateConstraint(term1_ad, verbose);
	constraintAD->initialize();
	constraintAN->addIntermediateConstraint(term1_an, verbose);
	constraintAN->initialize();

	/* evaluate constraint */
	Eigen::VectorXd g1_ad, g1_an;
	size_t g1_dim_ad, g1_dim_an;

	Eigen::Matrix<double, state_dim, 1> state; state.setRandom();
	Eigen::Matrix<double, input_dim, 1> input; input.setRandom();
	double time = 1.0;


	constraintAN->setCurrentStateAndControl(state, input, time);
	constraintAD->setCurrentStateAndControl(state, input, time);


	g1_an = constraintAN->evaluateIntermediate();
	g1_ad = constraintAD->evaluateIntermediate();

	// test if constraint violations are the same
	ASSERT_TRUE(g1_an.isApprox(g1_ad));

	Eigen::MatrixXd C_an, C_ad, C_cloned, C_cloned_an;
	Eigen::MatrixXd D_an, D_ad, D_cloned, D_cloned_an;
	size_t count = 0;

	C_an = constraintAN->jacobianStateIntermediate();
	C_ad = constraintAD->jacobianStateIntermediate();
	D_an = constraintAN->jacobianInputIntermediate();
	D_ad = constraintAD->jacobianInputIntermediate();


	std::shared_ptr<ct::optcon::ConstraintContainerAnalytical<state_dim, input_dim>> constraintAN_cloned (constraintAN->clone());

	std::shared_ptr<ct::optcon::ConstraintContainerAD<state_dim, input_dim>> constraintAD_cloned (constraintAD->clone());

	C_cloned_an = constraintAN_cloned->jacobianStateIntermediate();
	C_cloned = constraintAD_cloned->jacobianStateIntermediate();
	D_cloned_an = constraintAN_cloned->jacobianInputIntermediate();
	D_cloned = constraintAD_cloned->jacobianInputIntermediate();

	// compare jacobians
	ASSERT_TRUE(C_an.isApprox(C_ad));
	ASSERT_TRUE(C_an.isApprox(C_cloned));
	ASSERT_TRUE(C_an.isApprox(C_cloned_an));

	ASSERT_TRUE(D_an.isApprox(D_ad));
	ASSERT_TRUE(D_an.isApprox(D_cloned));
	ASSERT_TRUE(D_an.isApprox(D_cloned_an));
}

TEST(comparisonAnalyticAD, comparisonAnalyticAD)
{
	typedef ct::optcon::ConstraintBase<3, 3> Base;
	typedef CppAD::AD<CppAD::cg::CG<double> > ScalarCg;

	std::shared_ptr<ct::optcon::ConstraintContainerAD<3, 3>> constraintAD (
		new ct::optcon::ConstraintContainerAD<3, 3>());

	std::shared_ptr<ct::optcon::ConstraintContainerAnalytical<3, 3>> constraintAN (
		new ct::optcon::ConstraintContainerAnalytical<3, 3>());

	std::shared_ptr<ConstraintTerm1D<3, 3, ScalarCg>> term1_ad (
		new ConstraintTerm1D<3, 3, ScalarCg>()); term1_ad->setName("term1_ad");
	std::shared_ptr<ConstraintTerm2D<3, 3, ScalarCg>> term2_ad (
		new ConstraintTerm2D<3, 3, ScalarCg>()); term2_ad->setName("term2_ad");

	std::shared_ptr<ConstraintTerm1D<3, 3, double>> term1_an (
		new ConstraintTerm1D<3, 3, double>()); term1_an->setName("term1_an");
	std::shared_ptr<ConstraintTerm2D<3, 3, double>> term2_an (
		new ConstraintTerm2D<3, 3, double>()); term2_an->setName("term2_an");


	std::cout << "Adding terms to constraint_analytic" << std::endl;
	constraintAD->addIntermediateConstraint(term1_ad, verbose);
	constraintAD->addIntermediateConstraint(term2_ad, verbose);
	constraintAN->addIntermediateConstraint(term1_an, verbose);
	constraintAN->addIntermediateConstraint(term2_an, verbose);
	
	constraintAD->initialize();
	constraintAN->initialize();

	/* evaluate constraint */
	Eigen::VectorXd g1_ad, g1_an;
	size_t g1_dim_ad, g1_dim_an;

	Eigen::Vector3d state = Eigen::Vector3d::Random();
	Eigen::Vector3d control = Eigen::Vector3d::Random();
	double time = 0.5;

	constraintAN->setCurrentStateAndControl(state, control, time);
	constraintAD->setCurrentStateAndControl(state, control, time);

	g1_an = constraintAN->evaluateIntermediate();
	g1_ad = constraintAD->evaluateIntermediate();

	std::cout << "g1_an: " << g1_an.transpose() << std::endl;
	std::cout << "g1_ad: " << g1_ad.transpose() << std::endl;

	// test if constraint violations are the same
	ASSERT_TRUE(g1_an.isApprox(g1_ad));

	Eigen::MatrixXd C_an, C_ad, D_an, D_ad;
	C_an = constraintAN->jacobianStateIntermediate();
	D_an = constraintAN->jacobianInputIntermediate();
	C_ad = constraintAD->jacobianStateIntermediate();
	D_ad = constraintAD->jacobianInputIntermediate();

	ASSERT_TRUE(C_an.isApprox(C_ad));

	ASSERT_TRUE(D_an.isApprox(D_ad));

	ASSERT_TRUE(1.0);
}

} // namespace example
} // namespace optcon
} // namespace ct

#endif 
