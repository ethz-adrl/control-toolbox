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

#ifndef CT_OPTCON_COSTFUNCTION_QUADRATIC_SIMPLE_HPP_
#define CT_OPTCON_COSTFUNCTION_QUADRATIC_SIMPLE_HPP_

#include "CostFunctionQuadratic.hpp"

namespace ct{
namespace optcon{

/*!
 * \ingroup CostFunction
 *
 * \brief A simple quadratic cost function
 *
 * A simple, purely-quadratic cost function of the form
 * \f$ J(x,u,t) = \bar{x}^T Q \bar{x} + \bar{u}^T R \bar{u} + \bar{x}^T_f Q_f \bar{x}^T_f \f$
 * where \f$ \bar{x}, \bar{u} \f$ indicate deviations from a nominal (desired) state and control
 */
template <size_t STATE_DIM, size_t CONTROL_DIM,  typename SCALAR = double>
class CostFunctionQuadraticSimple : public CostFunctionQuadratic< STATE_DIM, CONTROL_DIM, SCALAR>
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM> 	state_matrix_t;
	typedef Eigen::Matrix<SCALAR, CONTROL_DIM, CONTROL_DIM> control_matrix_t;
	typedef Eigen::Matrix<SCALAR, CONTROL_DIM, STATE_DIM> 	control_state_matrix_t;

	typedef core::StateVector<STATE_DIM, SCALAR> 		state_vector_t;
	typedef core::ControlVector<CONTROL_DIM, SCALAR> 	control_vector_t;

	/**
	 * Constructs a simple, purely quadratic cost function with all zero elements.
	 */
	CostFunctionQuadraticSimple(){
		x_nominal_.setZero();
		Q_.setZero();
		u_nominal_.setZero();
		R_.setZero();
		x_final_.setZero();
		Q_final_.setZero();
		x_deviation_.setZero();
		u_deviation_.setZero();
	}

	/**
	 * Constructs a simple, purely quadratic cost function
	 * @param Q intermediate state cost weighting
	 * @param R intermediate control cost weighting
	 * @param x_nominal nominal (desired) state
	 * @param u_nominal nominal (desired) control
	 * @param x_final nominal (desired) final state
	 * @param Q_final final state cost weighting
	 */
	CostFunctionQuadraticSimple(const state_matrix_t& Q, const control_matrix_t& R,
			const state_vector_t& x_nominal, const control_vector_t& u_nominal,
			const state_vector_t& x_final, const state_matrix_t& Q_final) :
				x_nominal_(x_nominal),
				Q_(Q),
				u_nominal_(u_nominal),
				R_(R),
				x_final_(x_final),
				Q_final_(Q_final) {

		x_deviation_.setZero();
		u_deviation_.setZero();
	}

	virtual ~CostFunctionQuadraticSimple() {}


	CostFunctionQuadraticSimple(const CostFunctionQuadraticSimple& arg)
	:		x_deviation_(arg.x_deviation_),
	 		x_nominal_(arg.x_nominal_),
	 		Q_(arg.Q_),
	 		u_deviation_(arg.u_deviation_),
	 		u_nominal_(arg.u_nominal_),
	 		R_(arg.R_),
	 		x_final_(arg.x_final_),
	 		Q_final_(arg.Q_final_)
	{}


	/**
	 * Clones the cost function.
	 * @return
	 */
	CostFunctionQuadraticSimple<STATE_DIM, CONTROL_DIM, SCALAR>* clone () const override {
		return new CostFunctionQuadraticSimple<STATE_DIM, CONTROL_DIM, SCALAR>(*this);
	}

	virtual void setCurrentStateAndControl(const state_vector_t& x, const control_vector_t& u, const SCALAR& t) override {
		this->x_ = x;
		this->u_ = u;
		this->t_ = t;

		this->x_deviation_ = x - x_nominal_;
		this->u_deviation_ = u - u_nominal_;
	}

	virtual SCALAR evaluateIntermediate() override {
		SCALAR costQ = 0.5 * (x_deviation_.transpose() * Q_ * x_deviation_)(0);
		SCALAR costR = 0.5 * (u_deviation_.transpose() * R_ * u_deviation_)(0);
		return costQ + costR;
	}

	virtual state_vector_t stateDerivativeIntermediate() override {
		return  Q_ * x_deviation_;
	}

	virtual state_matrix_t stateSecondDerivativeIntermediate() override {
		return Q_;
	}

	virtual control_vector_t controlDerivativeIntermediate() override {
		return R_ * u_deviation_;
	}

	virtual control_matrix_t controlSecondDerivativeIntermediate() override {
		return R_;
	}

	virtual control_state_matrix_t stateControlDerivativeIntermediate() override {
		return control_state_matrix_t::Zero();
	}

	virtual SCALAR evaluateTerminal() override {
		state_vector_t x_deviation_final = this->x_ - x_final_;
		return 0.5 * x_deviation_final.transpose() * Q_final_ * x_deviation_final;
	}

	virtual state_vector_t stateDerivativeTerminal() override {
		state_vector_t x_deviation_final = this->x_ - x_final_;
		return Q_final_ * x_deviation_final;
	}

	virtual state_matrix_t stateSecondDerivativeTerminal() override {
		return Q_final_;
	}

	virtual void updateReferenceState(const state_vector_t& x_ref) override
	{
		x_nominal_ = x_ref;
	}

	virtual void updateFinalState(const state_vector_t& x_final) override
	{
		x_final_ = x_final;
	}



protected:
	state_vector_t x_deviation_;
	state_vector_t x_nominal_;
	state_matrix_t Q_;

	control_vector_t u_deviation_;
	control_vector_t u_nominal_;
	control_matrix_t R_;

	state_vector_t x_final_;
	state_matrix_t Q_final_;

};

} // namespace optcon
} // namespace ct

#endif /* CT_OPTCON_COSTFUNCTION_QUADRATIC_SIMPLE_HPP_ */
