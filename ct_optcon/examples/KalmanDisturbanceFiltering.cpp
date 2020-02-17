/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

/*!
 * \example KalmanDisturbanceFiltering.cpp
 *
 * This example shows how to use the Kalman Filter to simultaneously estimate the state and the disturbance acting 
 * on a simple linear oscillator system. The gist of the example is that the state is augmented with a disturbance 
 * and the system dynamics is augmented with the disturbance.
 * 
 * Using the parameters loaded from file, we can change initial state, apply additional feed-forward controls and 
 * vary both frequency and magnitude of the disturbance. Furthermore, it holds the Kalman Filter tuning parameters
 * and noise parameters for simulation.
 */

#include <ct/optcon/optcon.h>
#include "exampleDir.h"

// a damped oscillator has two states, position and velocity
const size_t state_dim = ct::core::SecondOrderSystem::STATE_DIM;      // = 2
const size_t control_dim = ct::core::SecondOrderSystem::CONTROL_DIM;  // = 1
const size_t output_dim = state_dim;                                  // we measure the full state
const size_t dist_dim = control_dim;                                  // we consider an input disturbance


/*!
 * \brief An example controller, in which we artifically add a time-varying disturbance.
 * @note this is only required for this simulated example. Not required for proper simulator/hardware setups.
 */
class CustomController : public ct::core::Controller<state_dim, control_dim>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const size_t state_dim = 2;
    static const size_t control_dim = 1;

    //! default constructor
    CustomController(const ct::core::ControlVector<control_dim>& uff_max,  // feedforward control amplitude
        const double& uff_frequency,         // frequency of the feedforward control osciallation
        const double& kp,                    // P gain
        const double& kd,                    // D gain
        const double& disturbance_max,       // disturbance amplitude
        const double& disturbance_frequency  // frequency of the disturbance
        )
        : uff_max_(uff_max),
          uff_frequency_(uff_frequency),
          kp_(kp),
          kd_(kd),
          d_(disturbance_max),
          d_freq_(disturbance_frequency)
    {
    }

    //! clone method, needs to be implemented, overrides ct::core::Controller::clone()
    CustomController* clone() const override { return new CustomController(*this); }
    //! override the compute control method with a custom control law which includes a disturbance
    void computeControl(const ct::core::StateVector<state_dim>& state,
        const double& t,
        ct::core::ControlVector<control_dim>& controlAction) override
    {
        controlAction = uff_max_ * std::sin(t * uff_frequency_);  // apply feedforward control
        controlAction += getSimulatedDisturbance(t);              // apply simulated disturbance
        controlAction(0) += -kp_ * state(0) - kd_ * state(1);     // add feedback control
    }

    //! simulate the disturbance allow to reconstruct it from outside
    ct::core::ControlVector<control_dim> getSimulatedDisturbance(const double& t)
    {
        ct::core::ControlVector<control_dim> dist_in;
        dist_in(0) = d_ * std::cos(t * t * d_freq_);
        return dist_in;
    }

private:
    ct::core::ControlVector<control_dim> uff_max_;  //! feedforward control action
    double uff_frequency_;                          // frequency of feedforward oscillation
    double kp_;                                     //! P gain
    double kd_;                                     //! D gain
    double d_;                                      //! disturbance magnitude
    double d_freq_;                                 //! frequency of the time-varying disturbance
};


int main(int argc, char** argv)
{
    // file with weights and settings
    std::string configFile = ct::optcon::exampleDir + "/kalmanDisturbanceFilterSettings.info";

    double dt;   // sampling time
    int nSteps;  // time horizon
    ct::core::loadScalar(configFile, "experiment_settings.dt", dt);
    ct::core::loadScalar(configFile, "experiment_settings.nSteps", nSteps);

    // state vector (initial state)
    ct::core::StateVector<state_dim> x;
    ct::core::loadMatrix(configFile, "x0", x);

    // create an oscillator with resonance-frequency w_n
    double w_n;
    ct::core::loadScalar(configFile, "experiment_settings.w_n", w_n);
    std::shared_ptr<ct::core::SecondOrderSystem> oscillator(new ct::core::SecondOrderSystem(w_n));

    // create our controller: a PD controller which also "simulates" a time-varying disturbance
    double kp = 10.0;
    double kd = 1.0;
    ct::core::ControlVector<control_dim> uff_magnitude;
    double uff_frequency, disturbance_max, disturbance_freq;
    ct::core::loadMatrix(configFile, "experiment_settings.uff_magnitude", uff_magnitude);
    ct::core::loadScalar(configFile, "experiment_settings.uff_frequency", uff_frequency);
    ct::core::loadScalar(configFile, "experiment_settings.disturbance_frequency", disturbance_freq);
    ct::core::loadScalar(configFile, "experiment_settings.disturbance_max", disturbance_max);

    std::shared_ptr<CustomController> disturbed_controller(
        new CustomController(uff_magnitude, uff_frequency, kp, kd, disturbance_max, disturbance_freq));

    // assign our controller to generate the disturbed data
    oscillator->setController(disturbed_controller);

    // create an integrator for "simulating" the measured data
    ct::core::Integrator<state_dim> integrator(oscillator, ct::core::IntegrationType::EULERCT);

    ct::core::StateVectorArray<state_dim> states;        // recorded sim states
    ct::core::ControlVectorArray<dist_dim> disturbance;  // recorded disturbance
    ct::core::tpl::TimeArray<double> times;              // recorded times

    // read process noise parameters from file
    ct::core::StateMatrix<state_dim> process_var;
    ct::core::loadMatrix(configFile, "process_noise.process_var", process_var);

    // process noise (scaled with dt)
    ct::core::GaussianNoise position_process_noise(0.0, dt * process_var(0, 0));
    ct::core::GaussianNoise velocity_process_noise(0.0, dt * process_var(1, 1));

    // simulate the disturbed system to generate data
    states.push_back(x);
    for (int i = 0; i < nSteps; i++)
    {
        // integrate system
        integrator.integrate_n_steps(x, i * dt, 1, dt);

        // log the disturbance (for plotting only)
        disturbance.push_back(disturbed_controller->getSimulatedDisturbance(dt * i));

        // add process noise
        position_process_noise.noisify(x(0));
        velocity_process_noise.noisify(x(1));

        // log state and time
        states.push_back(x);
        times.push_back(i * dt);
    }


    // observation model of the system dynamics (the model we "assume" to be correct, without disturbance)
    std::shared_ptr<ct::core::SecondOrderSystem> oscillator_obs(new ct::core::SecondOrderSystem(w_n));

    // create the controller that we "assumed" we were using (no disturbance)
    std::shared_ptr<CustomController> controller_nominal(
        new CustomController(uff_magnitude, uff_frequency, kp, kd, 0.0, 0.0));

    std::shared_ptr<ct::optcon::InputDisturbedSystem<state_dim, control_dim>> inputDisturbedSystem(
        new ct::optcon::InputDisturbedSystem<state_dim, control_dim>(oscillator_obs));

    // Observation matrix for the state
    ct::core::OutputStateMatrix<output_dim, state_dim> C;
    C.setIdentity();

    // Observation matrix for the disturbance (assuming the disturbance does not enter the output equation)
    ct::core::OutputStateMatrix<output_dim, dist_dim> Cd;
    Cd.setZero();

    // Total observation matrix, state and disturbance combined
    ct::core::OutputStateMatrix<output_dim, state_dim + dist_dim, double> Caug;
    Caug << C, Cd;

    // Kalman filter weighting matrices Q, dFdv and R
    ct::core::StateMatrix<state_dim + dist_dim> Qaug, dFdv;
    ct::core::OutputMatrix<output_dim> R;
    ct::core::loadMatrix(configFile, "kalman_weights.Qaug", Qaug);
    ct::core::loadMatrix(configFile, "kalman_weights.R", R);

    dFdv.setIdentity();

    // create a sensitivity approximator to obtain discrete-time dynamics matrices
    std::shared_ptr<ct::core::SystemLinearizer<state_dim + dist_dim, control_dim>> linearizer(
        new ct::core::SystemLinearizer<state_dim + dist_dim, control_dim>(inputDisturbedSystem));
    std::shared_ptr<ct::core::SensitivityApproximation<state_dim + dist_dim, control_dim>> sensApprox(
        new ct::core::SensitivityApproximation<state_dim + dist_dim, control_dim>(dt, linearizer));

    // set up the system model
    std::shared_ptr<ct::optcon::CTSystemModel<state_dim + dist_dim, control_dim>> sysModel(
        new ct::optcon::CTSystemModel<state_dim + dist_dim, control_dim>(inputDisturbedSystem, sensApprox, dFdv));

    // set up the measurement model
    std::shared_ptr<ct::optcon::LinearMeasurementModel<output_dim, state_dim + dist_dim>> measModel(
        new ct::optcon::LTIMeasurementModel<output_dim, state_dim + dist_dim>(Caug));

    // load measurement noise data from file
    ct::core::StateMatrix<state_dim> meas_var;
    ct::core::loadMatrix(configFile, "measurement_noise.measurement_var", meas_var);
    ct::core::GaussianNoise position_measurement_noise(0.0, meas_var(0, 0));
    ct::core::GaussianNoise velocity_measurement_noise(0.0, meas_var(1, 1));

    // generate initial state for the estimator (with noise, disturbance assumed to be zero at beginning)
    ct::core::StateVector<state_dim + dist_dim> x0aug;
    x0aug << states[0], 0.0;
    position_measurement_noise.noisify(x0aug(0));
    velocity_measurement_noise.noisify(x0aug(1));

    // data containers for logging data while estimating
    ct::core::StateVectorArray<state_dim + dist_dim> states_est(states.size());
    ct::core::OutputVectorArray<output_dim> output_meas(states.size());
    ct::core::StateMatrixArray<state_dim + dist_dim> cov_est(states.size());
    states_est[0] = x0aug;
    output_meas[0] = states[0];
    cov_est[0] = Qaug;


    // set up Extended Kalman Filter
    ct::optcon::ExtendedKalmanFilter<state_dim + dist_dim, control_dim, output_dim> ekf(
        sysModel, measModel, Qaug, R, x0aug, Qaug);


    // run the filter over the simulated data
    for (size_t i = 1; i < states.size(); ++i)
    {
        // compute an observation
        ct::core::OutputVector<output_dim> y = C * states[i] + Cd * disturbance[i - 1];
        position_measurement_noise.noisify(y(0));  // Position noise.
        velocity_measurement_noise.noisify(y(1));  // Velocity noise.
        output_meas[i] = y;

        // this is the control input that we would have "measured"
        ct::core::ControlVector<control_dim> nominal_control;
        controller_nominal->computeControl(states_est[i - 1].template head<state_dim>(), dt * (i - 1), nominal_control);

        // Kalman filter prediction step
        ekf.predict(nominal_control, dt, dt * i);

        // Kalman filter estimation step (state + disturbance)
        states_est[i] = ekf.update(y, dt, dt * i);

        cov_est[i] = ekf.getCovarianceMatrix();
    }


// plot if plotting library built.
#ifdef PLOTTING_ENABLED
    // some temporary containers for plotting
    std::vector<double> time_plot;
    std::vector<double> pos_est_plot, vel_est_plot, dist_est_plot;
    std::vector<double> pos_plot, vel_plot, dist_plot;
    std::vector<double> pos_meas_plot, vel_meas_plot;
    std::vector<double> pos_var_plot_upper, pos_var_plot_lower, vel_var_plot_upper, vel_var_plot_lower,
        dist_var_plot_upper, dist_var_plot_lower;

    for (size_t i = 0; i < times.size(); i++)
    {
        time_plot.push_back(times[i]);

        pos_plot.push_back(states[i](0));
        pos_meas_plot.push_back(output_meas[i](0));
        pos_est_plot.push_back(states_est[i](0));
        pos_var_plot_upper.push_back(pos_est_plot[i] + cov_est[i](0, 0));
        pos_var_plot_lower.push_back(pos_est_plot[i] - cov_est[i](0, 0));

        vel_plot.push_back(states[i](1));
        vel_meas_plot.push_back(output_meas[i](1));
        vel_est_plot.push_back(states_est[i](1));
        vel_var_plot_upper.push_back(vel_est_plot[i] + cov_est[i](1, 1));
        vel_var_plot_lower.push_back(vel_est_plot[i] - cov_est[i](1, 1));

        dist_plot.push_back(disturbance[i](0));
        dist_est_plot.push_back(states_est[i](2));
        dist_var_plot_upper.push_back(dist_est_plot[i] + cov_est[i](2, 2));
        dist_var_plot_lower.push_back(dist_est_plot[i] - cov_est[i](2, 2));
    }

    // plot position
    ct::core::plot::subplot(3, 1, 1);
    ct::core::plot::labelPlot("pos est", time_plot, pos_est_plot, "b");
    ct::core::plot::labelPlot("pvar_u", time_plot, pos_var_plot_upper, "b:");
    ct::core::plot::labelPlot("pvar_l", time_plot, pos_var_plot_lower, "b:");
    ct::core::plot::labelPlot("ground truth", time_plot, pos_plot, "r");
    ct::core::plot::labelPlot("pos meas", time_plot, pos_meas_plot, "kx--");
    ct::core::plot::legend();
    ct::core::plot::ylabel("pos [m]");

    // plot velocity
    ct::core::plot::subplot(3, 1, 2);
    ct::core::plot::labelPlot("vel est", time_plot, vel_est_plot, "b");
    ct::core::plot::labelPlot("vvar_u", time_plot, vel_var_plot_upper, "b:");
    ct::core::plot::labelPlot("vvar_l", time_plot, vel_var_plot_lower, "b:");
    ct::core::plot::labelPlot("ground truth", time_plot, vel_plot, "r");
    ct::core::plot::labelPlot("vel meas", time_plot, vel_meas_plot, "kx--");
    ct::core::plot::ylabel("vel [m/sec]");
    ct::core::plot::xlabel("time [sec]");
    ct::core::plot::legend();

    // plot disturbance
    ct::core::plot::subplot(3, 1, 3);
    ct::core::plot::labelPlot("dist est", time_plot, dist_est_plot, "b");
    ct::core::plot::labelPlot("dvar_u", time_plot, dist_var_plot_upper, "b:");
    ct::core::plot::labelPlot("dvar_l", time_plot, dist_var_plot_lower, "b:");
    ct::core::plot::labelPlot("ground truth", time_plot, dist_plot, "r");
    ct::core::plot::legend();
    ct::core::plot::ylabel("disturbance");

    ct::core::plot::show();

#else  // print results to command line
    for (size_t i = 0; i < times.size(); i++)
    {
        std::cout << std::fixed << std::setprecision(6) << "pos:\t" << states[i][0] << "\t pos est:\t"
                  << states_est[i][0] << "\t dist:\t" << states_est[i][2] << std::endl
                  << "vel: \t" << states[i][1] << "\t vel est:\t" << states_est[i][1] << std::endl
                  << std::endl;
    }
#endif

    return 0;
}
