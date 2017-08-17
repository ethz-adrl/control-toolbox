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


#ifndef CT_CORE_SIMPLE_SENSITIVITY_INTEGRATOR_CT_H_
#define CT_CORE_SIMPLE_SENSITIVITY_INTEGRATOR_CT_H_

#include <ct/core/systems/ControlledSystem.h>
#include <ct/core/systems/linear/LinearSystem.h>
#include <ct/core/integration/internal/SteppersCT.h>

namespace ct {
namespace core{


/**
 * @brief      This class can integrate a controlled system
 *             Furthermore, it provides first order derivatives with respect to
 *             initial state and control
 *
 * @tparam     STATE_DIM    The state dimension
 * @tparam     CONTROL_DIM  The control dimension
 * @tparam     SCALAR       The scalar type
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class SensitvityIntegrator : public Sensitivity<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef ct::core::StateVector<STATE_DIM, SCALAR> state_vector_t;
    typedef ct::core::ControlVector<CONTROL_DIM, SCALAR> control_vector_t;
    typedef Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM> state_matrix_t;
    typedef Eigen::Matrix<SCALAR, CONTROL_DIM, CONTROL_DIM> control_matrix_t;
    typedef Eigen::Matrix<SCALAR, STATE_DIM, CONTROL_DIM> state_control_matrix_t;

    typedef Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM + CONTROL_DIM> sensitivities_matrix_t;


    /**
     * @brief      Constructor
     *
     * @param[in]  system       The controlled system
     * @param[in]  stepperType  The integration stepper type
     */
    SensitvityIntegrator(
		const std::shared_ptr<ct::core::LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR> >& linearSystem,
		const std::shared_ptr<ct::core::Controller<STATE_DIM, CONTROL_DIM, SCALAR> >& controller,
        const ct::core::IntegrationType stepperType = ct::core::IntegrationType::EULERCT,
		const SCALAR dt) :
			dt_(dt),
			controller_(controller)
    {
    	setLinearSystem(linearSystem);
    	setStepper(stepperType);
    }


    /**
     * @brief      Destroys the object.
     */
    ~SensitvityIntegrator(){}

    /**
     * @brief      Initializes the steppers
     *
     * @param[in]  stepperType  The desired integration stepper type
     */
    void setStepper(const ct::core::IntegrationType stepperType)
    {
        switch(stepperType)
        {
            case ct::core::IntegrationType::EULERCT:
            {
                stepper_ = std::shared_ptr<ct::core::internal::StepperCTBase<sensitivities_matrix_t, SCALAR>>(
                    new ct::core::internal::StepperEulerCT<sensitivities_matrix_t, SCALAR>());
                break;
            }

            case ct::core::IntegrationType::RK4CT:
            {
                stepper_ = std::shared_ptr<ct::core::internal::StepperCTBase<sensitivities_matrix_t, SCALAR>>(
                    new ct::core::internal::StepperRK4CT<sensitivities_matrix_t, SCALAR>());
                break;
            }

            default:
                throw std::runtime_error("Invalid CT integration type");
        }
    }
    
    /**
     * @brief      Prepares the integrator to provide first order sensitivity
     *             generation by setting a linearsystem, enabling state caching
     *             and settings up the function objects
     *
     * @param[in]  linearSystem  The linearized system
     */
    void setLinearSystem(const std::shared_ptr<ct::core::LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>>& linearSystem)
    {
        linearSystem_ = linearSystem;
        
        dFdxDot_ = [this](const sensitivities_matrix_t& dX0In, sensitivities_matrix_t& dX0dt, const SCALAR t){

        	size_t i = round(t/dt_);

            state_matrix_t A = linearSystem_->getDerivativeState(*(this->x_)[i], *(this->u_)[i], t);
            state_control_matrix_t B = linearSystem_->getDerivativeControl(*(this->x_)[i], *(this->u_)[i], t);

            dX0dt.template leftCols<STATE_DIM>() = A * dX0In.template leftCols<STATE_DIM>();
            dX0dt.template rightCols<CONTROL_DIM>() = B * dX0In.template rightCols<CONTROL_DIM>() +
				B * controller_->getDerivativeU0(*(this->x_)[i], t);
        };
    }


    /**
     * @brief          Integrates the sensitivity ODE of the integrator with
     *                 respect to the initial state x0
     *
     * @param[in]      startTime  The start time
     * @param[in]      numSteps   The number of integration steps
     * @param[out]     A          Sensitivity with respect to state
     * @param[out]     B          Sensitivity with respect to control
     */
    void integrateSensitivity(
        const SCALAR startTime,
        const size_t numSteps,
        state_matrix_t& A,
		state_control_matrix_t& B,
        )
    {
        SCALAR time = startTime;

        A.setIdentity();
        B.setIdentity();

        sensitivities_matrix_t AB;
        AB << A, B;

        for(size_t i = 0; i < numSteps; ++i)
        {
            stepper_->do_step(dFdxDot_, AB, time, dt_);
            time += dt_;
        }

        A = AB.template leftCols<STATE_DIM>();
        B = AB.template rightCols<CONTROL_DIM>();
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
	{
		if (!(this->x_) || !(this->u_))
			throw std::runtime_error("SensitivityIntegrator.h: Cached trajectories not set.");
		if (this->x_.size() <=n || this->u_.size() <= n)
			throw std::runtime_error("SensitivityIntegrator.h: Cached trajectories too short.");

		assert(x == this->x_[n] && "cached trajectory does not match provided state");
		assert(u == this->u_[n] && "cached trajectory does not match provided input");

		integrateSensitivity(n*dt_,	1, A, B);
	};





private:
    double dt_;

    std::shared_ptr<ct::core::LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR> > linearSystem_;
    std::shared_ptr<ct::core::Controller<STATE_DIM, CONTROL_DIM, SCALAR> > controller_;

    // Sensitivities
    std::function<void (const sensitivities_matrix_t&, sensitivities_matrix_t&, const SCALAR)> dFdxDot_;

    std::shared_ptr<ct::core::internal::StepperCTBase<sensitivities_matrix_t, SCALAR>> stepper_;
};


}
}

#endif
