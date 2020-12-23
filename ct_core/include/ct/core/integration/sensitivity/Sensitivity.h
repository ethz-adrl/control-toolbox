/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/systems/LinearSystem.h>

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


template <typename MANIFOLD>
class Sensitivity : public LinearSystem<MANIFOLD, DISCRETE_TIME>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using SCALAR = typename MANIFOLD::Scalar;
    using Base = LinearSystem<MANIFOLD, DISCRETE_TIME>;
    using Time_t = typename Base::Time_t;
    using control_vector_t = typename Base::control_vector_t;
    using state_matrix_t = typename Base::state_matrix_t;
    using state_control_matrix_t = typename Base::state_control_matrix_t;

    typedef std::shared_ptr<DiscreteArray<MANIFOLD>> StateVectorArrayPtr;
    typedef std::shared_ptr<ControlVectorArray<SCALAR>> ControlVectorArrayPtr;


    Sensitivity() : xSubstep_(nullptr), uSubstep_(nullptr) {}
    virtual ~Sensitivity() {}
    virtual Sensitivity<MANIFOLD>* clone() const override
    {
        throw std::runtime_error("clone not implemented for Sensitivity");
    }

    virtual void setLinearSystem(
        const std::shared_ptr<LinearSystem<MANIFOLD, CONTINUOUS_TIME>>& linearSystem) = 0;

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

    virtual const state_matrix_t& getDerivativeState(const MANIFOLD& m,
        const control_vector_t& u,
        const Time_t t = Time_t(0.0)) override = 0;

    virtual const state_control_matrix_t& getDerivativeControl(const MANIFOLD& m,
        const control_vector_t& u,
        const Time_t t = Time_t(0.0)) override = 0;

    virtual void getDerivatives(state_matrix_t& A,
        state_control_matrix_t& B,
        const MANIFOLD& x,
        const MANIFOLD& x_next,
        const control_vector_t& u,
        const size_t nSubsteps,
        const Time_t n = Time_t(0)) override = 0;

    virtual void getDerivatives(state_matrix_t& A,
        state_control_matrix_t& B,
        const MANIFOLD& x,
        const control_vector_t& u,
        const Time_t n = Time_t(0)) override = 0;

protected:
    std::vector<StateVectorArrayPtr, Eigen::aligned_allocator<StateVectorArrayPtr>>* xSubstep_;
    std::vector<ControlVectorArrayPtr, Eigen::aligned_allocator<ControlVectorArrayPtr>>* uSubstep_;
};

}  // namespace core
}  // namespace ct
