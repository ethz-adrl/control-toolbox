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

#ifndef INCLUDE_CT_CORE_SYSTEMS_LINEAR_LINEARSYSTEMDISCRETIZER_H_
#define INCLUDE_CT_CORE_SYSTEMS_LINEAR_LINEARSYSTEMDISCRETIZER_H_

#include <unsupported/Eigen/MatrixFunctions>

namespace ct {
namespace core {

//! interface class for a general linear system or linearized system
/*!
 * Defines the interface for a linear system
 *
 * \tparam STATE_DIM size of state vector
 * \tparam CONTROL_DIM size of input vector
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class LinearSystemDiscretizer : public DiscreteLinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum class Approximation {
		FORWARD_EULER = 0,
		BACKWARD_EULER = 1,
		TUSTIN = 2,
		MATRIX_EXPONENTIAL = 3
	};

	typedef typename Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM> state_matrix_t; //!< state Jacobian type
	typedef typename Eigen::Matrix<SCALAR, STATE_DIM, CONTROL_DIM> state_control_matrix_t; //!< input Jacobian type

	//! default constructor
	/*!
	 * @param type system type
	 */
	LinearSystemDiscretizer(
			const std::shared_ptr<LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>>& linearSystem,
			const SCALAR& dt,
			const Approximation& approximation = Approximation::FORWARD_EULER,
			const ct::core::SYSTEM_TYPE& type = ct::core::SYSTEM_TYPE::GENERAL):
		DiscreteLinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>(type),
		dt_(dt),
		approximation_(approximation)
	{}

	LinearSystemDiscretizer(const LinearSystemDiscretizer<STATE_DIM, CONTROL_DIM, SCALAR>& other) :
		linearSystem_(other.linearSystem_.clone()),
		dt_(other.dt_),
		approximation_(other.approximation_)
	{
	}

	//! destructor
	virtual ~LinearSystemDiscretizer(){};

	//! deep cloning
	virtual LinearSystemDiscretizer<STATE_DIM, CONTROL_DIM, SCALAR>* clone() {
		return new LinearSystemDiscretizer<STATE_DIM, CONTROL_DIM, SCALAR>(*this);
	}

	void setApproximation(const Approximation& approximation) { approximation_ = approximation; }

	Approximation getApproximation() const { return approximation_; }

	virtual void getAandB(
			const StateVector<STATE_DIM, SCALAR>& x,
			const int n,
			const ControlVector<CONTROL_DIM, SCALAR>& u,
			state_matrix_t& A,
			state_control_matrix_t& B) override
	{
		switch(approximation_)
		{
			case Approximation::FORWARD_EULER:
			{
				A = state_matrix_t::Identity();
				A += dt_ * linearSystem_->getDerivativeState(x, u, n*dt_);
				B = dt_ * linearSystem_->getDerivativeControl(x, u, n*dt_);
				break;
			}
			case Approximation::BACKWARD_EULER:
			{
				state_matrix_t aNew = dt_ * linearSystem_->getDerivativeState(x, u, n*dt_);
				A = (state_matrix_t::Identity() -  aNew).colPivHouseholderQr().inverse();
				B = A * dt_ * linearSystem_->getDerivativeControl(x, u, n*dt_);
				break;
			}
			case Approximation::TUSTIN:
			{
				// todo: this is probably not correct
				state_matrix_t aNew = 0.5 * dt_ * linearSystem_->getDerivativeState(x, u, n*dt_);
				state_matrix_t aNewInv = (state_matrix_t::Identity() -  aNew).colPivHouseholderQr().inverse();
				A = aNewInv * (state_matrix_t::Identity() + aNew);
				B = aNewInv * dt_ * linearSystem_->getDerivativeControl(x, u, n*dt_);
				break;
			}
			case Approximation::MATRIX_EXPONENTIAL:
			{
				state_matrix_t Ac = linearSystem_->getDerivativeState(x, u, dt_*n);
				state_matrix_t Adt = dt_ * Ac;

				A = Adt.exp();
				B = Ac.inverse() * (A - state_matrix_t::Identity()) *  linearSystem_->getDerivativeControl(x, u, dt_*n);
				break;
			}
		}
	}

private:
	std::shared_ptr<LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>> linearSystem_;
	SCALAR dt_;
	Approximation approximation_;
};



}
}



#endif /* INCLUDE_CT_CORE_SYSTEMS_LINEAR_LINEARSYSTEMDISCRETIZER_H_ */
