/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

/*!
 * \example KalmanFiltering.cpp
 *
 * This example shows how to use the Kalman Filter to estimate the state of a simple oscillator.
 *
 */

#include <ct/optcon/optcon.h>
#include <CustomController.h>  // Using the custom controller from ct_core examples.

int main(int argc, char** argv)
{
    // a damped oscillator has two states, position and velocity
    const size_t STATE_DIM = ct::core::SecondOrderSystem::STATE_DIM;      // = 2
    const size_t CONTROL_DIM = ct::core::SecondOrderSystem::CONTROL_DIM;  // = 1
    const size_t OUTPUT_DIM = 1;  // we assume we observe a one-dimensional output

    // create an initial state: we initialize it at a point with unit deflection and zero velocity
    ct::core::StateVector<STATE_DIM> x;
    x(0) = 1.0;
    x(1) = 0.0;

    // create an oscillator
    double w_n = 50;
    std::shared_ptr<ct::core::SecondOrderSystem> oscillator(new ct::core::SecondOrderSystem(w_n));

    // create a simple PD controller
    double kp = 10;
    double kd = 1;
    ct::core::ControlVector<CONTROL_DIM> uff;
    uff << 2.0;
    std::shared_ptr<CustomController> controller(new CustomController(uff, kp, kd));

    // assign the controller
    oscillator->setController(controller);

    // create an integrator for "simulating" the measured data
    ct::core::Integrator<STATE_DIM> integrator(oscillator, ct::core::IntegrationType::RK4);

    ct::core::StateVectorArray<STATE_DIM> states;
    ct::core::tpl::TimeArray<double> times;

    // simulate 100 steps
    double dt = 0.001;
    double t0 = 0.0;
    size_t nSteps = 100;
    integrator.integrate_n_steps(x, t0, nSteps, dt, states, times);

    // create system observation matrix C: we measure a combination of position and velocity.
    ct::core::OutputStateMatrix<OUTPUT_DIM, STATE_DIM> C;
    C << 0.5, 1;

    // create Kalman Filter weighting matrices
    ct::core::StateMatrix<STATE_DIM, double> Q = ct::core::StateMatrix<STATE_DIM, double>::Identity();
    ct::core::OutputMatrix<OUTPUT_DIM, double> R = 10 * ct::core::OutputMatrix<OUTPUT_DIM, double>::Identity();

    // create a sensitivity approximator to compute A and B matrices
    std::shared_ptr<ct::core::SystemLinearizer<STATE_DIM, CONTROL_DIM>> linearizer(
        new ct::core::SystemLinearizer<STATE_DIM, CONTROL_DIM>(oscillator));
    ct::core::SensitivityApproximation<STATE_DIM, CONTROL_DIM> sensApprox(dt, linearizer);

    // set up Extended Kalman Filter
    ct::optcon::ExtendedKalmanFilter<STATE_DIM> ekf(states[0]);
    ct::optcon::StateObserver<OUTPUT_DIM, STATE_DIM, CONTROL_DIM, ct::optcon::ExtendedKalmanFilter<STATE_DIM>>
        stateObserver(oscillator, sensApprox, dt, C, ekf, Q, R);

    ct::core::GaussianNoise noise(0, 0.1);

    ct::core::StateVectorArray<STATE_DIM> states_est;
    states_est.push_back(states[0]);
    ct::core::StateVector<STATE_DIM> xt;

    // run the filter over the simulated data
    for (size_t i = 1; i < states.size(); ++i)
    {
        // compute an observation
        xt = states[i];
        noise.noisify(xt[0]);  // Position noise.
        noise.noisify(xt[1]);  // Velocity noise.
        ct::core::OutputVector<OUTPUT_DIM> y = C * xt;

        // Kalman filter prediction step
        stateObserver.predict();

        // Kalman filter estimation step
        ct::core::StateVector<STATE_DIM> x_est = stateObserver.update(y);

        // and log for printing
        states_est.push_back(x_est);
    }

    // print results
    for (size_t i = 0; i < states_est.size(); ++i)
        std::cout << "State\t\tState_est\n"
                  << std::fixed << std::setprecision(6) << states[i][0] << "\t" << states_est[i][0] << std::endl
                  << states[i][1] << "\t" << states_est[i][1] << std::endl
                  << std::endl;

    return 0;
}
