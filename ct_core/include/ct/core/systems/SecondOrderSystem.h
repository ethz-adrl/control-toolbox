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

//! Describes a continuous-time damped oscillator
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
class SecondOrderSystem : public ControlledSystem<EuclideanState<2, SCALAR>, 1, CONTINUOUS_TIME>
{
public:
    static const size_t STATE_DIM = 2;    //!< state dimension (position, velocity)
    static const size_t CONTROL_DIM = 1;  //!< control dimension (force)

    using StateVector_t = EuclideanState<STATE_DIM>;
    using Base = ControlledSystem<EuclideanState<2, SCALAR>, 1, CONTINUOUS_TIME>;
    using Controller_t = typename Base::Controller_t;
    using Time_t = typename Base::Time_t;

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
        std::shared_ptr<Controller_t> controller = nullptr);

    //! copy constructor
    SecondOrderSystem(const SecondOrderSystem& arg);

    //! constructor using a more mechanical definition (spring-mass-damping)
    /*!
	 * @param k spring stiffness
	 * @param m mass
	 * @param d damper constant
	 * @param g_dc DC input gain
	 * @param controller controller (optional)
	 */
    SecondOrderSystem(SCALAR k, SCALAR m, SCALAR d, SCALAR g_dc, std::shared_ptr<Controller_t> controller = nullptr);

    //! deep copy
    SecondOrderSystem* clone() const override;

    //! destructor
    virtual ~SecondOrderSystem();

    //! set the dynamics
    /*!
	 * @param w_n eigenfrequency
	 * @param zeta damping ratio
	 * @param g_dc DC gain
	 */
    void setDynamics(SCALAR w_n, SCALAR zeta = SCALAR(1.0), SCALAR g_dc = SCALAR(1.0));

    //! evaluate the system dynamics
    /*!
	 * @param state current state (position, velocity)
	 * @param t current time (gets ignored)
	 * @param control control action
	 * @param derivative derivative (velocity, acceleration)
	 */
    virtual void computeControlledDynamics(const StateVector_t& state,
        const Time_t& t,
        const ControlVector<1, SCALAR>& control,
        typename StateVector_t::Tangent& derivative) override;

    //! check the parameters
    /*!
	 * @return true if parameters are physical
	 */
    bool checkParameters();

    //! print out infos about the system
    void printSystemInfo();

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
