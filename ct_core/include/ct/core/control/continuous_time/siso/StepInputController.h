/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {

//! A simple step input
/*!
 * A step input controller with a standard heaviside form
 *
 * \f[
 *
 *
 * \begin{aligned}
 * u(t) \mapsto
 * \begin{cases}
 * 0 : & t < t_{step} \\
 * g : & t \ge t_{step}
 * \end{cases}
 * \end{aligned}
 *
 * \f]
 *
 * where \f$ g \f$ is a constant gain and \f$ t_{step} \f$ is the time of the step.
 */
class StepInputController : public Controller<1, 1, double>
{
public:
    //! Parameters of the step input function
    /*!
     * Contains the constant gain \f$ g \f$ and the time of the step \f$ t_{step} \f$
     */
    struct Parameters
    {
        Parameters(double gain_ = 1.0, double t_step_ = 1.0) : gain(gain_), t_step(t_step_) {}
        double gain;  //! gain
        Time t_step;  //! time of step
    };

    //! default constructor
    StepInputController(const Parameters& parameters = Parameters()) : parameters_(parameters) {}
    //! copy constructor
    StepInputController(const StepInputController& arg) : parameters_(arg.parameters_) {}
    //! deep cloning
    StepInputController* clone() const override { return new StepInputController(*this); }
    //! computes control input
    /*!
     * Computes the control input. The state parameter gets ignored.
     * @param state current state (ignored)
     * @param t current time
     * @return control action, either 0 or g
     */
    void computeControl(const StateVector<1, double>& state,
        const double& t,
        ControlVector<1, double>& controlAction) override
    {
        controlAction(0) = parameters_.gain * (t >= parameters_.t_step);
    }

private:
    Parameters parameters_;  //! parameters of the step function
};
}  // namespace core
}  // namespace ct
