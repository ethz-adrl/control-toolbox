/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <cmath>
#include <memory>
#include <iostream>

#include "ControlledSystem.h"

namespace ct {
namespace core {

namespace tpl {

//! Describes a damped oscillator
/*!
 * System dynamics definition of a damped oscillator with the state
 *
 * \f[
 *  x = [p ~ \dot{p}]^T
 * \f]
 *
 *
 * dynamics
 *
 * \f[
 * 	 f(x,u,t) = \dot{x} = [\dot{p} ~ \ddot{p}]^T =
 *
 * 	 \begin{bmatrix}
 * 	 	\dot{p} \\
 * 	    g_{dc} \omega^2_{n} u - 2.0  \zeta  \omega_n  \dot{p} - \omega^2_{n}  p;
 *
 * 	 \end{bmatrix}
 *
 * \f]
 *
 * where \f$ g_{dc} \f$ is a dc gain on the input.
 *
 * If interpreted as a mechanical oscillator
 *
 * \f[
 *  \omega_n = \sqrt{\frac{k}{m}}
 * \f]
 *
 * is the eigenfrequency and
 *
 * \f[
 *   \zeta = \frac{d}{2 \sqrt{m k}}
 * \f]
 *
 * is the damping ratio, given mass \f$ m \f$, spring stiffness \f$ k \f$ and damper constant \f$ d \f$.
 *
 * An example of how to evaluate the dynamics of this oscillator is provided in unit test \ref SecondOrderSystemTest.cpp
 *
 * \todo rename to damped oscillator
 */
template <typename SCALAR>
class SecondOrderSystem : public ControlledSystem<2, 1, SCALAR>
{
public:
    static const size_t STATE_DIM = 2;    //!< state dimension (position, velocity)
    static const size_t CONTROL_DIM = 1;  //!< control dimension (force)

    typedef ControlledSystem<2, 1, SCALAR> Base;
    typedef typename Base::time_t time_t;

    //! default constructor
    SecondOrderSystem() = delete;

    //! constructor directly using frequency and damping coefficients
    /*!
	 *
	 * @param w_n eigenfrequency
	 * @param zeta damping ratio
	 * @param g_dc DC gain on input, set to \f$ w_n^2 \f$ to achieve amplification 1
	 * @param controller controller (optional)
	 */
    SecondOrderSystem(SCALAR w_n,
        SCALAR zeta = SCALAR(1.0),
        SCALAR g_dc = SCALAR(1.0),
        std::shared_ptr<Controller<2, 1, SCALAR>> controller = nullptr)
        : ControlledSystem<2, 1, SCALAR>(controller, SYSTEM_TYPE::SECOND_ORDER),
          w_n_(w_n),
          w_n_square_(w_n_ * w_n_),
          zeta_(zeta),
          g_dc_(g_dc)
    {
    }

    //! copy constructor
    SecondOrderSystem(const SecondOrderSystem& arg)
        : ControlledSystem<2, 1, SCALAR>(arg),
          w_n_(arg.w_n_),
          w_n_square_(arg.w_n_square_),
          zeta_(arg.zeta_),
          g_dc_(arg.g_dc_)
    {
    }

    //! constructor using a more mechanical definition (spring-mass-damping)
    /*!
	 * @param k spring stiffness
	 * @param m mass
	 * @param d damper constant
	 * @param g_dc DC input gain
	 * @param controller controller (optional)
	 */
    SecondOrderSystem(SCALAR k, SCALAR m, SCALAR d, SCALAR g_dc, std::shared_ptr<Controller<2, 1>> controller = nullptr)
        : ControlledSystem<2, 1>(controller),
          w_n_(std::sqrt(k / m)),
          w_n_square_(w_n_ * w_n_),
          zeta_(d / (2.0 * m * k)),
          g_dc_(g_dc)
    {
    }

    //! deep copy
    SecondOrderSystem* clone() const override { return new SecondOrderSystem(*this); }
    //! destructor
    virtual ~SecondOrderSystem() {}
    //! set the dynamics
    /*!
	 * @param w_n eigenfrequency
	 * @param zeta damping ratio
	 * @param g_dc DC gain
	 */
    void setDynamics(SCALAR w_n, SCALAR zeta = SCALAR(1.0), SCALAR g_dc = SCALAR(1.0))
    {
        w_n_ = w_n;
        w_n_square_ = w_n_ * w_n_;
        zeta_ = zeta;
        g_dc_ = g_dc;
    }

    //! evaluate the system dynamics
    /*!
	 * @param state current state (position, velocity)
	 * @param t current time (gets ignored)
	 * @param control control action
	 * @param derivative derivative (velocity, acceleration)
	 */
    virtual void computeControlledDynamics(const StateVector<2, SCALAR>& state,
        const time_t& t,
        const ControlVector<1, SCALAR>& control,
        StateVector<2, SCALAR>& derivative) override
    {
        derivative(0) = state(1);
        derivative(1) = g_dc_ * control(0) - 2.0 * zeta_ * w_n_ * state(1) - w_n_square_ * state(0);
    }

    //! check the parameters
    /*!
	 * @return true if parameters are physical
	 */
    bool checkParameters()
    {
        if (zeta_ < 0)
        {
            std::cout << "Warning: Damping is negative!" << std::endl;
            return false;
        }
        if (w_n_ < 0)
        {
            std::cout << "Warning: Frequency w_n is negative!" << std::endl;
            return false;
        }
        if (g_dc_ < 0)
        {
            std::cout << "Warning: Steady state gain is negative!" << std::endl;
            return false;
        }
        if (g_dc_ == 0)
        {
            std::cout << "Warning: Steady state gain is zero!" << std::endl;
            return false;
        }

        return true;
    }

    //! print out infos about the system
    void printSystemInfo()
    {
        std::cout << "Frequency: " << w_n_ << std::endl;
        std::cout << "Zeta: " << zeta_ << std::endl;
        std::cout << "DC gain: " << g_dc_ << std::endl;

        std::cout << "System is ";
        if (zeta_ == 0.0)
        {
            std::cout << "undamped" << std::endl;
        }
        if (zeta_ == 1.0)
        {
            std::cout << "critically damped" << std::endl;
        }
        if (zeta_ > 1.0)
        {
            std::cout << "overdamped" << std::endl;
        }
        if (zeta_ < 1.0)
        {
            std::cout << "underdamped" << std::endl;
        }
    }

private:
    SCALAR w_n_;         //!< eigenfrequency
    SCALAR w_n_square_;  //!< eigenfrequency squared
    SCALAR zeta_;        //!< damping ratio
    SCALAR g_dc_;        //!< input DC gain
};

}  // namespace tpl

typedef tpl::SecondOrderSystem<double> SecondOrderSystem;  //!< harmonic oscillator (double)

}  // namespace core
}  // namespace ct
