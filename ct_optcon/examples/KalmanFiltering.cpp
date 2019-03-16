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
#include "exampleDir.h"

int main(int argc, char** argv)
{
    // file with kalman weights
    std::string pathToWeights = ct::optcon::exampleDir + "/kalmanFilterWeights.info";

    // a damped oscillator has two states, position and velocity
    const size_t STATE_DIM = ct::core::SecondOrderSystem::STATE_DIM;      // = 2
    const size_t CONTROL_DIM = ct::core::SecondOrderSystem::CONTROL_DIM;  // = 1
    const size_t OUTPUT_DIM = 2;  // we assume we observe the full state (however with noise)

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
    ct::core::ControlVectorArray<CONTROL_DIM> controls;
    ct::core::tpl::TimeArray<double> times;

    ct::core::StateMatrix<STATE_DIM> process_var;
    ct::core::loadMatrix(pathToWeights, "process_noise.process_var", process_var);

    ct::core::GaussianNoise position_process_noise(0.0, process_var(0, 0));
    ct::core::GaussianNoise velocity_process_noise(0.0, process_var(1, 1));

    // simulate 100 steps
    double dt = 0.001;
    size_t nSteps = 100;
    states.push_back(x);
    for (size_t i = 0; i < nSteps; i++)
    {
        // compute control (needed for filter later)
        ct::core::ControlVector<CONTROL_DIM> u_temp;
        controller->computeControl(x, i * dt, u_temp);
        controls.push_back(u_temp);

        integrator.integrate_n_steps(x, i * dt, 1, dt);

        position_process_noise.noisify(x(0));  // Position noise.
        velocity_process_noise.noisify(x(1));  // Velocity noise.

        states.push_back(x);
        times.push_back(i * dt);
    }

    // create system observation matrix C: we measure both position and velocity
    ct::core::OutputStateMatrix<OUTPUT_DIM, STATE_DIM> C;
    C.setIdentity();

    // load Kalman Filter weighting matrices from file
    ct::core::StateMatrix<STATE_DIM> Q, dFdv;
    ct::core::OutputMatrix<OUTPUT_DIM> R;
    ct::core::loadMatrix(pathToWeights, "kalman_weights.Q", Q);
    ct::core::loadMatrix(pathToWeights, "kalman_weights.R", R);
    std::cout << "Loaded Kalman R as " << std::endl << R << std::endl;
    std::cout << "Loaded Kalman Q as " << std::endl << Q << std::endl;

    dFdv.setIdentity();  // todo tune me!

    // create a sensitivity approximator to compute A and B matrices
    std::shared_ptr<ct::core::SystemLinearizer<STATE_DIM, CONTROL_DIM>> linearizer(
        new ct::core::SystemLinearizer<STATE_DIM, CONTROL_DIM>(oscillator));
    ct::core::SensitivityApproximation<STATE_DIM, CONTROL_DIM> sensApprox(dt, linearizer);

    // set up Extended Kalman Filter
    ct::optcon::ExtendedKalmanFilter<STATE_DIM> ekf(states[0], R);

    // the observer is supplied with a dynamic model identical to the one used above for data generation
    std::shared_ptr<ct::core::SecondOrderSystem> oscillator_observer_model(new ct::core::SecondOrderSystem(w_n));

    // initialize the observer
    ct::optcon::StateObserver<OUTPUT_DIM, STATE_DIM, CONTROL_DIM, ct::optcon::ExtendedKalmanFilter<STATE_DIM>>
        stateObserver(oscillator_observer_model, sensApprox, C, R, ekf, Q, R, dFdv);


    ct::core::StateMatrix<STATE_DIM> meas_var;
    ct::core::loadMatrix(pathToWeights, "measurement_noise.measurement_var", meas_var);

    ct::core::GaussianNoise position_measurement_noise(0.0, meas_var(0, 0));
    ct::core::GaussianNoise velocity_measurement_noise(0.0, meas_var(1, 1));

    ct::core::StateVectorArray<STATE_DIM> states_est(states.size());
    ct::core::StateVectorArray<STATE_DIM> states_meas(states.size());
    states_est[0] = states[0];
    states_meas[0] = states[0];

    // run the filter over the simulated data
    for (size_t i = 1; i < states.size(); ++i)
    {
        // compute an observation
        states_meas[i] = states[i];

        // todo this is technically not correct, the noise enters not on the state but on the output!!!
        position_measurement_noise.noisify(states_meas[i](0));  // Position noise.
        velocity_measurement_noise.noisify(states_meas[i](1));  // Velocity noise.
        ct::core::OutputVector<OUTPUT_DIM> y = C * states_meas[i];

        // Kalman filter prediction step
        stateObserver.predict(controls[i], dt, dt * i);

        // Kalman filter estimation step
        ct::core::StateVector<STATE_DIM> x_est = stateObserver.update(y, dt, dt * i);

        // and log for printing
        states_est[i] = x_est;
    }


// plot if plotting library built.
#ifdef PLOTTING_ENABLED

    std::vector<double> time_plot, pos_est_plot, vel_est_plot, pos_plot, vel_plot, pos_meas_plot, vel_meas_plot;
    for (size_t i = 0; i < times.size(); i++)
    {
        time_plot.push_back(times[i]);
        pos_est_plot.push_back(states_est[i](0));
        vel_est_plot.push_back(states_est[i](1));
        pos_plot.push_back(states[i](0));
        vel_plot.push_back(states[i](1));
        pos_meas_plot.push_back(states_meas[i](0));
        vel_meas_plot.push_back(states_meas[i](1));
    }

    // plot position
    ct::core::plot::subplot(2, 1, 1);
    ct::core::plot::labelPlot("pos est", time_plot, pos_est_plot);
    ct::core::plot::labelPlot("ground truth", time_plot, pos_plot, "r--");
    ct::core::plot::labelPlot("pos meas", time_plot, pos_meas_plot, "k--");
    ct::core::plot::legend();
    ct::core::plot::ylabel("pos [m]");

    // plot velocity
    ct::core::plot::subplot(2, 1, 2);
    ct::core::plot::labelPlot("vel est", time_plot, vel_est_plot);
    ct::core::plot::labelPlot("ground truth", time_plot, vel_plot, "r--");
    ct::core::plot::labelPlot("vel meas", time_plot, vel_meas_plot, "k--");
    ct::core::plot::ylabel("vel [m/sec]");
    ct::core::plot::xlabel("time [sec]");
    ct::core::plot::legend();

    ct::core::plot::show();
#else  // print results to command line
    for (size_t i = 0; i < states_est.size(); ++i)
        std::cout << "State\t\tState_est\n"
                  << std::fixed << std::setprecision(6) << states[i][0] << "\t" << states_est[i][0] << std::endl
                  << states[i][1] << "\t" << states_est[i][1] << std::endl
                  << std::endl;
#endif

    return 0;
}
