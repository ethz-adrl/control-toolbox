/*
 * Sensitivity.hpp
 *
 *  Created on: Aug 17, 2017
 *      Author: neunertm
 */

#ifndef INCLUDE_CT_CORE_INTEGRATION_SENSITIVITY_SENSITIVITY_HPP_
#define INCLUDE_CT_CORE_INTEGRATION_SENSITIVITY_SENSITIVITY_HPP_

namespace ct {
namespace core {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class Sensitivity :  public DiscreteLinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>
{

public:
	Sensitivity() :
		x_(nullptr),
		u_(nullptr)
	{}

	virtual ~Sensitivity() {}

	/*!
	 * Set the trajectory reference for linearization. This should also include potential substeps that the integrator produces.
	 * @param x
	 * @param u
	 */
	void setSubstepTrajectoryReference(const StateVectorArray<STATE_DIM, SCALAR>& xSubstep, const ControlVectorArray<STATE_DIM, SCALAR>& uSubstep)
	{
		x_ = &x;
		u_ = &u;
	}

	/*!
	 * retrieve discrete-time linear system matrices A and B.
	 * @param x	the state setpoint
	 * @param u the control setpoint
	 * @param n the time setpoint
	 * @param A the resulting linear system matrix A
	 * @param B the resulting linear system matrix B
	 */
	virtual void getAandB(
			const StateVector<STATE_DIM, SCALAR>& x,
			const ControlVector<CONTROL_DIM, SCALAR>& u,
			const int n,
			state_matrix_t& A,
			state_control_matrix_t& B) override
	= 0;

private:
	StateVectorArray<STATE_DIM, SCALAR>* x_;
	ControlVectorArray<STATE_DIM, SCALAR>* u_;

	StateVectorArray<STATE_DIM, SCALAR>* xSubstep_;
	ControlVectorArray<STATE_DIM, SCALAR>* uSubstep_;

};


}
}


#endif /* INCLUDE_CT_CORE_INTEGRATION_SENSITIVITY_SENSITIVITY_HPP_ */
