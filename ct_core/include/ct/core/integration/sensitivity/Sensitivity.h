/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {

//! settings for the SensitivityApproximation
struct SensitivityApproximationSettings
{
    //! different discrete-time approximations to linear systems
    enum class APPROXIMATION
    {
        FORWARD_EULER = 0,
        BACKWARD_EULER,
        SYMPLECTIC_EULER,
        TUSTIN,
        MATRIX_EXPONENTIAL
    };

    SensitivityApproximationSettings(double dt, APPROXIMATION approx) : dt_(dt), approximation_(approx) {}
    //! discretization time-step
    double dt_;

    //! type of discretization strategy used.
    APPROXIMATION approximation_;
};

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class Sensitivity : public DiscreteLinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef std::shared_ptr<StateVectorArray<STATE_DIM, SCALAR>> StateVectorArrayPtr;
    typedef std::shared_ptr<ControlVectorArray<CONTROL_DIM, SCALAR>> ControlVectorArrayPtr;


    Sensitivity() : xSubstep_(nullptr), uSubstep_(nullptr) {}
    virtual ~Sensitivity() {}
    virtual Sensitivity<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override
    {
        throw std::runtime_error("clone not implemented for Sensitivity");
    }

    virtual void setLinearSystem(const std::shared_ptr<LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>>& linearSystem) = 0;

    //! update the time discretization
    virtual void setTimeDiscretization(const SCALAR& dt) = 0;

    //! update the approximation type for the discrete-time system
    virtual void setApproximation(const SensitivityApproximationSettings::APPROXIMATION& approx) {}
    /*!
	 * Set the trajectory reference for linearization. This should also include potential substeps that the integrator produces.
	 * @param x
	 * @param u
	 */
    void setSubstepTrajectoryReference(
        std::vector<StateVectorArrayPtr, Eigen::aligned_allocator<StateVectorArrayPtr>>* xSubstep,
        std::vector<ControlVectorArrayPtr, Eigen::aligned_allocator<ControlVectorArrayPtr>>* uSubstep)
    {
        xSubstep_ = xSubstep;
        uSubstep_ = uSubstep;
    }

    /*!
	 * retrieve discrete-time linear system matrices A and B.
	 * @param x	the state setpoint
	 * @param u the control setpoint
	 * @param n the time setpoint
	 * @param numSteps number of timesteps of trajectory for which to get the sensitivity for
	 * @param A the resulting linear system matrix A
	 * @param B the resulting linear system matrix B
	 */
    virtual void getAandB(const StateVector<STATE_DIM, SCALAR>& x,
        const ControlVector<CONTROL_DIM, SCALAR>& u,
        const StateVector<STATE_DIM, SCALAR>& x_next,
        const int n,
        size_t numSteps,
        StateMatrix<STATE_DIM, SCALAR>& A,
        StateControlMatrix<STATE_DIM, CONTROL_DIM, SCALAR>& B) override = 0;

protected:
    std::vector<StateVectorArrayPtr, Eigen::aligned_allocator<StateVectorArrayPtr>>* xSubstep_;
    std::vector<ControlVectorArrayPtr, Eigen::aligned_allocator<ControlVectorArrayPtr>>* uSubstep_;
};
}
}
