/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <limits>

namespace ct {
namespace core {

//! A standard PIDController
/*!
 * Implements a standard PID feedback law with anti wind-up:
 *
 * \f[
 *  u(x,t) = -k_p (x-x_d) - k_d (\dot{x} - \dot{x}_d) - k_i \int_0^t (x-x_d) dt
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
template <typename SCALAR = double>
class PIDController
{
public:
    //! Parameters of a PID Controller
    /*!
     * Contains the gains as well as the limits and saturation
     */
    struct parameters_t
    {
        parameters_t(SCALAR kp = SCALAR(0.0),
            SCALAR ki = SCALAR(0.0),
            SCALAR kd = SCALAR(0.0),
            Time dt_ = 0.01,
            SCALAR Imax_ = std::numeric_limits<SCALAR>::max(),
            SCALAR uMax_ = std::numeric_limits<SCALAR>::max(),
            SCALAR uMin_ = -std::numeric_limits<SCALAR>::max())
            : k_p(kp), k_i(ki), k_d(kd), dt(dt_), Imax(Imax_), uMax(uMax_), uMin(uMin_)
        {
        }
        SCALAR k_p;   //! proportional gain
        SCALAR k_i;   //! integral gain
        SCALAR k_d;   //! differential gain
        SCALAR dt;    //! timestep for I-part
        SCALAR Imax;  //! anti-windup I-part saturation
        SCALAR uMax;  //! max u output
        SCALAR uMin;  //! min u output
    };

    //! setpoint for the PID controller
    /*!
     * Contains both the desired state and derivative setpoint
     */
    struct setpoint_t
    {
        setpoint_t(SCALAR stateDes = SCALAR(0.0), SCALAR stateDevDes = SCALAR(0.0))
            : stateDesired_(stateDes), stateDerivativeDesired_(stateDevDes)
        {
        }

        SCALAR stateDesired_;            //! desired state
        SCALAR stateDerivativeDesired_;  //! desired derivative
    };

    //! default constructor
    /*!
     * Initializes setpoints and parameters to their default values
     * @param parameters The parameters for the PID controller
     * @param setpoint The setpoint for the PID controller
     */
    PIDController(const parameters_t& parameters = parameters_t(), const setpoint_t& setpoint = setpoint_t());

    //! copy constructor
    PIDController(const PIDController& other);

    //! destructor
    virtual ~PIDController();

    //! clone operator
    PIDController* clone() const;
    //! set the initial state
    /*!
     * Set the initial state for finite differencing for the D-part
     * @param state the initial state
     */
    void setInitialState(const SCALAR& state);
    //! set the state
    /*!
     * @param state The setpoint to set
     */
    void setDesiredState(const SCALAR& state);
    //! set the desired state and derivative
    /*!
     * @param state the desired state
     * @param stateDerivative the desired state derivative
     */
    void setDesiredState(const SCALAR& state, const SCALAR& stateDerivative);

    //! computes the control input based on the current state and time
    /*!
     * uses finite-differences for computing the state derivative
     * @param state the current state
     * @param t the current time
     * @return the control input
     */
    SCALAR computeControl(const SCALAR& state, const Time& t);

    //! computes the control input
    /*!
     *
     * @param state the current state
     * @param stateDerivative the current state derivative
     * @param t the current time
     * @return the control input
     */
    SCALAR computeControl(const SCALAR& state, const SCALAR& stateDerivative, const Time& t);

    //! change the control parameters
    /*!
     *
     * @param parameters the new PID controller parameters
     */
    void changeParameters(const parameters_t& parameters);
    //! returns a reference to the parameters
    /*!
     * Changes to the parameters can be directly made through this reference
     * @return reference to the parameters
     */
    parameters_t& Parameters();
    //! resets the controller
    /*!
     * resets the I part as well as the previous state
     */
    void reset();

protected:
    SCALAR statePrevious_;  //! the previous state used for finite differences
    SCALAR I_;              //! the current I part (integrated)

    parameters_t parameters_;  //! the parameters of the PID controller
    setpoint_t setpoint_;      //! the setpoint of the PID controller

private:
    //! saturates the control according to the boundaries
    void saturateControl(SCALAR& u);

    //! computes the I-part
    void computeI(const SCALAR& error);
};

}  // namespace core
}  // namespace ct
