/**********************************************************************************************************************
This file is part of the Control Toobox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <limits>
#include "SISOControllerBase.h"

namespace ct {
namespace core {

//! A standard PIDController
/*!
 * Implements a standard PID feedback law with anti wind-up:
 *
 * \f[
 * 	u(x,t) = -k_p (x-x_d) - k_d (\dot{x} - \dot{x}_d) - k_i \int_0^t (x-x_d) dt
 * \f]
 *
 * subject to the saturation
 *
 * \f[
 *  u_{min} \leq u(x,t) \leq u_{max}
 * \f]
 *
 * - The state derivative \f$\dot{x}\f$ can be provided or is computed using finite differences between current and previous state
 * - The anti-windup ensures that the I part stays within fixed boundaries, i.e. saturates
 */
class PIDController : public SISOControllerBase
{
public:
    //! Parameters of a PID Controller
    /*!
	 * Contains the gains as well as the limits and saturation
	 */
    struct parameters_t
    {
        parameters_t(double kp = 0.0,
            double ki = 0.0,
            double kd = 0.0,
            double dt_ = 0.01,
            double Imax_ = std::numeric_limits<double>::max(),
            double uMax_ = std::numeric_limits<double>::max(),
            double uMin_ = -std::numeric_limits<double>::max())
            : k_p(kp), k_i(ki), k_d(kd), dt(dt_), Imax(Imax_), uMax(uMax_), uMin(uMin_)
        {
        }
        double k_p;   //! proportional gain
        double k_i;   //! integral gain
        double k_d;   //! differential gain
        double dt;    //! timestep for I-part
        double Imax;  //! anti-windup I-part saturation
        double uMax;  //! max u output
        double uMin;  //! min u output
    };

    //! setpoint for the PID controller
    /*!
	 * Contains both the desired state and derivative setpoint
	 */
    struct setpoint_t
    {
        setpoint_t(double stateDes = 0.0, double stateDevDes = 0.0)
            : stateDesired_(stateDes), stateDerivativeDesired_(stateDevDes)
        {
        }

        double stateDesired_;            //! desired state
        double stateDerivativeDesired_;  //! desired derivative
    };

    //! default constructor
    /*!
	 * Initializes setpoints and parameters to their default values
	 * @param parameters The parameters for the PID controller
	 * @param setpoint The setpoint for the PID controller
	 */
    PIDController(const parameters_t& parameters = parameters_t(), const setpoint_t& setpoint = setpoint_t())
        : statePrevious_(0.0), I_(0.0), parameters_(parameters), setpoint_(setpoint){};

    //! copy constructor
    PIDController(const PIDController& other)
        : statePrevious_(other.statePrevious_), I_(other.I_), parameters_(other.parameters_), setpoint_(other.setpoint_)
    {
    }

    //! detailed constructor
    virtual ~PIDController(){};

    //! clone operator
    PIDController* clone() const override { return new PIDController(*this); }
    //! set the initial state
    /*!
	 * Set the initial state for finite differencing for the D-part
	 * @param state the initial state
	 */
    void setInitialState(const double& state) { statePrevious_ = state; }
    //! set the state
    /*!
	 * @param state The setpoint to set
	 */
    void setDesiredState(const double& state) { setpoint_.stateDesired_ = state; }
    //! set the desired state and derivative
    /*!
	 * @param state the desired state
	 * @param stateDerivative the desired state derivative
	 */
    void setDesiredState(const double& state, const double& stateDerivative)
    {
        setpoint_.stateDesired_ = state;
        setpoint_.stateDerivativeDesired_ = stateDerivative;
    }

    //! computes the control input based on the current state and time
    /*!
	 * uses finite-differences for computing the state derivative
	 * @param state the current state
	 * @param t the current time
	 * @return the control input
	 */
    double computeControl(const double& state, const core::Time& t) override;

    //! computes the control input
    /*!
	 *
	 * @param state the current state
	 * @param stateDerivative the current state derivative
	 * @param t the current time
	 * @return the control input
	 */
    double computeControl(const double& state, const double& stateDerivative, const core::Time& t);

    //! change the control parameters
    /*!
	 *
	 * @param parameters the new PID controller parameters
	 */
    void changeParameters(const parameters_t& parameters) { parameters_ = parameters; }
    //! returns a reference to the parameters
    /*!
	 * Changes to the parameters can be directly made through this reference
	 * @return reference to the parameters
	 */
    parameters_t& Parameters() { return parameters_; }
    //! resets the controller
    /*!
	 * resets the I part as well as the previous state
	 */
    void reset();

protected:
    double statePrevious_;  //! the previous state used for finite differences
    double I_;              //! the current I part (integrated)

    parameters_t parameters_;  //! the parameters of the PID controller
    setpoint_t setpoint_;      //! the setpoint of the PID controller

private:
    //! saturates the control according to the boundaries
    void saturateControl(double& u);

    //! computes the I-part
    void computeI(double error);
};

}  // namespace core
}  // namespace ct
