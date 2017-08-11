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

#ifndef LTISYSTEM_H_
#define LTISYSTEM_H_

namespace ct {
namespace core {

//! Linear time-invariant system
/*!
 * This defines a general linear time-invariant system of the form
 *
 * \f[
 *
 *  \begin{aligned}
 *  \dot{x} &= Ax + Bu \\
 *  y &= Cx + Du
 *  \end{aligned}
 *
 * \f]
 *
 * \tparam STATE_DIM size of state vector
 * \tparam CONTROL_DIM size of control vector
 */
template <size_t STATE_DIM, size_t CONTROL_DIM>
class LTISystem : public LinearSystem<STATE_DIM, CONTROL_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	//! Constructs a linear time invariant system
	/*!
	 * @param A A matrix
	 * @param B B matrix
	 * @param C C matrix
	 * @param D D matrix
	 * @return instance of the LTI system
	 */
	LTISystem(
		const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& A,
		const Eigen::Matrix<double, STATE_DIM, CONTROL_DIM>& B,
		const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& C = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity(),
		const Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> D = Eigen::Matrix<double, STATE_DIM, CONTROL_DIM>::Zero()
	) :
		A_(A),
		B_(B),
		C_(C),
		D_(D)
	{}

	//! copy constructor
	LTISystem(const LTISystem& arg):
			A_(arg.A_),
			B_(arg.B_),
			C_(arg.C_),
			D_(arg.D_)
	{}

	//! deep clone
	LTISystem<STATE_DIM, CONTROL_DIM>* clone() const override {
		return new LTISystem<STATE_DIM, CONTROL_DIM>(*this);
	}

	//! get A matrix
	virtual const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& getDerivativeState(const StateVector<STATE_DIM>& x, const ControlVector<CONTROL_DIM>& u, const double t = 0.0) override {
		return A_;
	}

	//! get B matrix
	virtual const Eigen::Matrix<double, STATE_DIM, CONTROL_DIM>& getDerivativeControl(const StateVector<STATE_DIM>& x, const ControlVector<CONTROL_DIM>& u, const double t = 0.0) override {
		return B_;
	}

	//! get A matrix
	Eigen::Matrix<double, STATE_DIM, STATE_DIM>& A() { return A_; }

	//! get B matrix
	Eigen::Matrix<double, STATE_DIM, CONTROL_DIM>& B() { return B_; }

	//! get C matrix
	Eigen::Matrix<double, STATE_DIM, STATE_DIM>& C() { return C_; }

	//! get D matrix
	Eigen::Matrix<double, STATE_DIM, CONTROL_DIM>& D() { return D_; }

	//! computes the system dynamics
	/*!
	 * Computes \f$ \dot{x} = Ax + Bu \f$
	 * @param state current state x
	 * @param t time (gets ignored)
	 * @param control control input
	 * @param derivative state derivative
	 */
	void computeControlledDynamics(
		const Eigen::Matrix<double, STATE_DIM, 1>& state,
		const Time& t,
		const Eigen::Matrix<double, CONTROL_DIM, 1>& control,
		Eigen::Matrix<double, STATE_DIM, 1>& derivative
	)
	{
		derivative = A_ * state + B_ * control;
	}

	//! computes the system output (measurement)
	/*!
	 * Computes \f$ y = Cx + Du \f$
	 * @param state current state x
	 * @param t time (gets ignored)
	 * @param control control input
	 * @param output system output (measurement)
	 */
	void computeOutput(
		const Eigen::Matrix<double, STATE_DIM, 1>& state,
		const Time& t,
		const Eigen::Matrix<double, CONTROL_DIM, 1>& control,
		Eigen::Matrix<double, STATE_DIM, 1>& output
	)
	{
		output = C_ * state + D_ * control;
	}

	//! computes the controllability matrix
	/*!
	 * Computes the controllability matrix to assess controllability. See isControllable() for the full test.
	 *
	 * \todo Move to LinearSystem
	 *
	 * @param CO controllability matrix
	 */
	void computeControllabilityMatrix(Eigen::Matrix<double, STATE_DIM, STATE_DIM*CONTROL_DIM>& CO)
	{
		CO.block<STATE_DIM, CONTROL_DIM>(0, 0) = B_;

		for (size_t i=1; i<STATE_DIM; i++)
		{
			CO.block<STATE_DIM, CONTROL_DIM>(0, i*CONTROL_DIM) = A_*CO.block<STATE_DIM, CONTROL_DIM>(0, (i-1) * CONTROL_DIM);
		}
	}

	//! checks if system is fully controllable
	/*!
	 * \todo Move to LinearSystem
	 *
	 * @return true if fully controllable, false if only partially or non-controllable
	 */
	bool isControllable()
	{
		Eigen::Matrix<double, STATE_DIM, STATE_DIM*CONTROL_DIM> CO;
		computeControllabilityMatrix(CO);

		Eigen::FullPivLU< Eigen::Matrix<double, STATE_DIM, STATE_DIM*CONTROL_DIM> > LUdecomposition(CO);
		return LUdecomposition.rank() == STATE_DIM;
	}

	//! computes the observability matrix
	/*!
	 * \todo Move to LinearSystem
	 *
	 * Computes the observability matrix to assess observability. See isObservable() for the full test.
	 * @param O observability matrix
	 */
	void computeObservabilityMatrix(Eigen::Matrix<double, STATE_DIM, STATE_DIM*STATE_DIM>& O)
	{
		O.block<STATE_DIM, STATE_DIM>(0, 0) = C_;

		for (size_t i=1; i<STATE_DIM; i++)
		{
			O.block<STATE_DIM, STATE_DIM>(i*STATE_DIM, 0) = O.block<STATE_DIM, STATE_DIM>(0, (i-1)*STATE_DIM) * A_;
		}
	}

	//! checks if system is fully observable
	/*!
	 * \todo Move to LinearSystem
	 *
	 * @return true if fully observable, false if only partially or non-observable
	 */
	bool isObservable()
	{
		Eigen::Matrix<double, STATE_DIM, STATE_DIM*STATE_DIM> O;
		computeObservabilityMatrix(O);

		Eigen::FullPivLU< Eigen::Matrix<double, STATE_DIM, STATE_DIM*STATE_DIM> > LUdecomposition(O);
		return LUdecomposition.rank() == STATE_DIM;
	}


private:

	Eigen::Matrix<double, STATE_DIM, STATE_DIM> A_; //!< A matrix
	Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> B_; //!< B matrix

	Eigen::Matrix<double, STATE_DIM, STATE_DIM> C_; //!< C matrix
	Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> D_; //!< D matrix
};

}
}

#endif /* LTISYSTEM_H_ */
