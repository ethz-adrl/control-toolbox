/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
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
const size_t STATE_DIM = ct::core::SecondOrderSystem::STATE_DIM;      // = 2
const size_t CONTROL_DIM = ct::core::SecondOrderSystem::CONTROL_DIM;  // = 1
const size_t OUTPUT_DIM = STATE_DIM;                                  // we measure the full state
const size_t DIST_DIM = CONTROL_DIM;                                  // we consider an input disturbance


/*!
 * \brief An example controller, in which we artifically add a time-varying disturbance.
 * @note this is only required for this simulated example. Not required for proper simulator/hardware setups.
 */
class CustomController : public ct::core::Controller<STATE_DIM, CONTROL_DIM>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const size_t STATE_DIM = 2;
    static const size_t CONTROL_DIM = 1;

    //! default constructor
    CustomController(const ct::core::ControlVector<CONTROL_DIM>& uff_max,  // feedforward control amplitude
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
    void computeControl(const ct::core::StateVector<STATE_DIM>& state,
        const double& t,
        ct::core::ControlVector<CONTROL_DIM>& controlAction) override
    {
        controlAction = uff_max_ * std::sin(t * uff_frequency_);  // apply feedforward control
        controlAction += getSimulatedDisturbance(t);              // apply simulated disturbance
        controlAction(0) += -kp_ * state(0) - kd_ * state(1);     // add feedback control
    }

    //! simulate the disturbance allow to reconstruct it from outside
    ct::core::ControlVector<CONTROL_DIM> getSimulatedDisturbance(const double& t)
    {
        ct::core::ControlVector<CONTROL_DIM> dist_in;
        dist_in(0) = d_ * std::cos(t * t * d_freq_);
        return dist_in;
    }

private:
    ct::core::ControlVector<CONTROL_DIM> uff_max_;  //! feedforward control action
    double uff_frequency_;                          // frequency of feedforward oscillation
    double kp_;                                     //! P gain
    double kd_;                                     //! D gain
    double d_;                                      //! disturbance magnitude
    double d_freq_;                                 //! frequency of the time-varying disturbance
};


/*!
 * Implementation of the "DisturbedSystem" which is going to be used for handed over to the Kalman Filter
 * for dynamics prediction and computing derivatives.
 * @note this system is not used for simulation, but for filtering.
 */
template <size_t STATE_DIM, size_t DIST_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class CustomDisturbedSystem : public ct::optcon::DisturbedSystem<STATE_DIM, DIST_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! constructor
    CustomDisturbedSystem(std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>> sys)
        : ct::optcon::DisturbedSystem<STATE_DIM, DIST_DIM, CONTROL_DIM, SCALAR>(sys->getController()), system_(sys)
    {
    }

    //! copy constructor
    CustomDisturbedSystem(const CustomDisturbedSystem& other)
        : ct::optcon::DisturbedSystem<STATE_DIM, DIST_DIM, CONTROL_DIM, SCALAR>(*this), system_(other.system_->clone())
    {
    }

    //! deep cloning
    CustomDisturbedSystem* clone() const override { return new CustomDisturbedSystem(*this); }

    /*!
     * Override the computeControlledDynamics() with a custom update rule.
     */
    void computeControlledDynamics(const ct::core::StateVector<STATE_DIM + DIST_DIM, SCALAR>& state,
        const SCALAR& t,
        const ct::core::ControlVector<CONTROL_DIM, SCALAR>& control,
        ct::core::StateVector<STATE_DIM + DIST_DIM, SCALAR>& derivative) override
    {
        derivative.setZero();
        ct::core::StateVector<STATE_DIM, SCALAR> tempDerivative;

        // the control consists of actual commanded control plus the estimated input disturbance,
        // which is the augmented part of the state vector
        ct::core::ControlVector<CONTROL_DIM, SCALAR> disturbed_control = control + state.template tail<DIST_DIM>();

        // the dynamics of the augmented system
        system_->computeControlledDynamics(state.head(STATE_DIM), t, disturbed_control, tempDerivative);
        derivative.template head<STATE_DIM>() = tempDerivative;
    }


private:
    // the nominal system (the one we are trying to control)
    std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>> system_;
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
    ct::core::StateVector<STATE_DIM> x;
    ct::core::loadMatrix(configFile, "x0", x);

    // create an oscillator with resonance-frequency w_n
    double w_n;
    ct::core::loadScalar(configFile, "experiment_settings.w_n", w_n);
    std::shared_ptr<ct::core::SecondOrderSystem> oscillator(new ct::core::SecondOrderSystem(w_n));

    // create our controller: a PD controller which also "simulates" a time-varying disturbance
    double kp = 10.0;
    double kd = 1.0;
    ct::core::ControlVector<CONTROL_DIM> uff_magnitude;
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
    ct::core::Integrator<STATE_DIM> integrator(oscillator, ct::core::IntegrationType::EULERCT);

    ct::core::StateVectorArray<STATE_DIM> states;        // recorded sim states
    ct::core::ControlVectorArray<DIST_DIM> disturbance;  // recorded disturbance
    ct::core::tpl::TimeArray<double> times;              // recorded times

    // read process noise parameters from file
    ct::core::StateMatrix<STATE_DIM> process_var;
    ct::core::loadMatrix(configFile, "process_noise.process_var", process_var);

    // process noise (scaled with dt)
    ct::core::GaussianNoise position_process_noise(0.0, dt * process_var(0, 0));
    ct::core::GaussianNoise velocity_process_noise(0.0, dt * process_var(1, 1));

    // simulate the disturbed system to generate data
    ct::core::Time t0 = 0.0;
    states.push_back(x);
    for (size_t i = 0; i < nSteps; i++)
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

    std::shared_ptr<CustomDisturbedSystem<STATE_DIM, DIST_DIM, CONTROL_DIM>> customdisturbedSystem(
        new CustomDisturbedSystem<STATE_DIM, DIST_DIM, CONTROL_DIM>(oscillator_obs));

    // Observation matrix for the state
    ct::core::OutputStateMatrix<OUTPUT_DIM, STATE_DIM> C;
    C.setIdentity();

    // Observation matrix for the disturbance (assuming the disturbance does not enter the output equation)
    ct::core::OutputStateMatrix<OUTPUT_DIM, DIST_DIM> Cd;
    Cd.setZero();

    // Total observation matrix, state and disturbance combined
    ct::core::OutputStateMatrix<OUTPUT_DIM, STATE_DIM + DIST_DIM, double> Caug;
    Caug << C, Cd;

    // Kalman filter weighting matrices Q, dFdv and R
    ct::core::StateMatrix<STATE_DIM + DIST_DIM> Qaug, dFdv;
    ct::core::OutputMatrix<OUTPUT_DIM> R;
    ct::core::loadMatrix(configFile, "kalman_weights.Qaug", Qaug);
    ct::core::loadMatrix(configFile, "kalman_weights.R", R);

    dFdv.setIdentity();  // todo tune me

    // create a sensitivity approximator to obtain discrete-time dynamics matrices
    std::shared_ptr<ct::core::SystemLinearizer<STATE_DIM + DIST_DIM, CONTROL_DIM>> linearizer(
        new ct::core::SystemLinearizer<STATE_DIM + DIST_DIM, CONTROL_DIM>(customdisturbedSystem));
    ct::core::SensitivityApproximation<STATE_DIM + DIST_DIM, CONTROL_DIM> sensApprox(dt, linearizer);

    // load measurement noise data from file
    ct::core::StateMatrix<STATE_DIM> meas_var;
    ct::core::loadMatrix(configFile, "measurement_noise.measurement_var", meas_var);
    ct::core::GaussianNoise position_measurement_noise(0.0, meas_var(0, 0));
    ct::core::GaussianNoise velocity_measurement_noise(0.0, meas_var(1, 1));

    // generate initial state for the estimator (with noise, disturbance assumed to be zero at beginning)
    ct::core::StateVector<STATE_DIM + DIST_DIM> x0aug;
    x0aug << states[0], 0.0;
    position_measurement_noise.noisify(x0aug(0));
    velocity_measurement_noise.noisify(x0aug(1));

    // create instance of the extended Kalman Filter
    // initialize it estimated starting covariance equal to Qaug
    ct::optcon::ExtendedKalmanFilter<STATE_DIM + DIST_DIM> ekf(x0aug, Qaug);

    // create instance of the disturbance observer
    ct::optcon::DisturbanceObserver<OUTPUT_DIM, STATE_DIM, DIST_DIM, CONTROL_DIM,
        ct::optcon::ExtendedKalmanFilter<STATE_DIM + DIST_DIM>>
        disturbanceObserver(customdisturbedSystem, sensApprox, Caug, ekf, Qaug, R, dFdv);


    // data containers for logging data while estimating
    ct::core::StateVectorArray<STATE_DIM + DIST_DIM> states_est(states.size());
    ct::core::OutputVectorArray<OUTPUT_DIM> output_meas(states.size());
    ct::core::StateMatrixArray<STATE_DIM + DIST_DIM> cov_est(states.size());
    states_est[0] = x0aug;
    output_meas[0] = states[0];
    cov_est[0] = disturbanceObserver.getCovarianceMatrix();


    // run the filter over the simulated data
    for (size_t i = 1; i < states.size(); ++i)
    {
        // compute an observation
        ct::core::OutputVector<OUTPUT_DIM> y = C * states[i] + Cd * disturbance[i - 1];
        position_measurement_noise.noisify(y(0));  // Position noise.
        velocity_measurement_noise.noisify(y(1));  // Velocity noise.
        output_meas[i] = y;

        // this is the control input that we would have "measured"
        ct::core::ControlVector<CONTROL_DIM> nominal_control;
        controller_nominal->computeControl(states_est[i - 1].template head<STATE_DIM>(), dt * (i - 1), nominal_control);

        // Kalman filter prediction step
        disturbanceObserver.predict(nominal_control, dt, dt * i);

        // Kalman filter estimation step (state + disturbance)
        states_est[i] = disturbanceObserver.update(y, dt, dt * i);

        cov_est[i] = disturbanceObserver.getCovarianceMatrix();
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
    std::cout << "State\t\tState_est\tdisturbance_est\n"
              << std::fixed << std::setprecision(6) << states[i][0] << "\t" << states_est[i][0] << "\t"
              << states_est[i][2] << std::endl
              << states[i][1] << "\t" << states_est[i][1] << std::endl
              << std::endl;
#endif

    return 0;
}
