/*!
 * \brief This example shows how to define your own system.
 *
 * \note This generates a "System", not a "ControlledSystem". If you wanted to use
 * this system for controller design, you should derive from "ControlledSystem"
 *
 * \example Masspoint.h
 */

#pragma once

#include <ct/core/core.h>  // as usual, include CT

// create a class that derives from ct::core::System
class Masspoint : public ct::core::System<2>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    static const size_t STATE_DIM = 2;

    // constructor
    Masspoint(double mass, double d) : mass_(mass), d_(d) {}

    // copy constructor
    Masspoint(const Masspoint& other) : mass_(other.mass_), d_(other.d_) {}

    // destructor
    ~Masspoint() = default;

    // clone method for deep copying
    Masspoint* clone() const override
    {
        return new Masspoint(*this);  // calls copy constructor
    }

    // The system dynamics. We override this method which gets called by e.g. the Integrator
    void computeDynamics(const ct::core::StateVector<STATE_DIM>& x,
        const ct::core::Time& t,
        ct::core::StateVector<STATE_DIM>& derivative) override
    {
        // first part of state derivative is the velocity
        derivative(0) = x(1);

        // second part is the acceleration which is caused by damper forces
        derivative(1) = -d_ / mass_ * x(1);
    }

private:
    double mass_;
    double d_;
};
