/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

/*!
 * A simple custom PD controller for an oscillator
 *
 * see \ref DampedOscillatorCustomController.cpp on how to use it.
 *
 * \example CustomController.h
 */
//! an example controller class that takes a 2-dimensional state and outputs a 1-dimensional control action
class CustomController : public ct::core::Controller<2, 1>
{
public:
    static const size_t state_dim = 2;    // two states
    static const size_t control_dim = 1;  // one control action

    //! default constructor
    CustomController(const ct::core::ControlVector<control_dim>& uff,  // feedforward control
        const double& kp,                                              // P gain
        const double& kd                                               // D gain
        )
        : uff_(uff), kp_(kp), kd_(kd)
    {
    }

    //! destructor
    ~CustomController() {}
    //! copy constructor
    CustomController(const CustomController& other) : uff_(other.uff_), kp_(other.kp_), kd_(other.kd_) {}
    //! clone method, needs to be implemented, overrides ct::core::Controller::clone()
    CustomController* clone() const override
    {
        return new CustomController(*this);  // calls copy constructor
    }

    //! override the compute control method with a custom control law
    void computeControl(const ct::core::StateVector<state_dim>& state,
        const double& t,
        ct::core::ControlVector<control_dim>& controlAction) override
    {
        controlAction = uff_;                                 // apply feedforward control
        controlAction(0) -= kp_ * state(0) + kd_ * state(1);  // add feedback control
    }

private:
    ct::core::ControlVector<control_dim> uff_;  //! feedforward control action
    double kp_;                                 //! P gain
    double kd_;                                 //! D gain
};
