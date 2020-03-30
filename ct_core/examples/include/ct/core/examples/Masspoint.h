/*!
 * \brief This example shows how to define your own system.
 *
 * The system created here is a simple mass-point second-order dynamics in one dimension.
 * Therefore, the state dimension is 2, and the control dimension is 1
 * 
 * \note This generates a "System" in continuous-time, not a "ControlledSystem". 
 * If you wanted to use this system for controller design, you should derive from "ControlledSystem" 
 * instead.
 *
 * \example Masspoint.h
 */

#pragma once

#include <ct/core/core.h>

// create a class that derives from ct::core::System in continuous time
class Masspoint : public ct::core::System<ct::core::EuclideanState<2>, ct::core::CONTINUOUS_TIME>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // convenience definitions, with state-space dimension 2. Define control dimension=1
    static constexpr size_t state_dim = 2;
    using State = ct::core::EuclideanState<state_dim>;
    using StateDerivative = State::Tangent;

    // constructor
    Masspoint(const double mass, const double damping) : mass_(mass), d_(damping) {}
    // copy constructor (required)
    Masspoint(const Masspoint& other) : mass_(other.mass_), d_(other.d_) {}
    // clone method for deep copying (required)
    Masspoint* clone() const override
    {
        return new Masspoint(*this);  // calls copy constructor
    }

    // The system dynamics. We override this method, which gets called by e.g. the Integrator,
    // or by the system linearizers
    void computeDynamics(const State& x, const double& time, StateDerivative& dxdt) override
    {
        // first part of state derivative is the velocity
        dxdt(0) = x(1);

        // second part is the acceleration which is caused by damper forces
        dxdt(1) = -d_ / mass_ * x(1);
    }

private:
    double mass_;
    double d_;
};
