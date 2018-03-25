/*!
 * \example KalmanFiltering.cpp
 *
 * This example shows how to use the Kalman Filter to estimate the state of a simple oscillator.
 *
 */

#include <chrono>
#include <iomanip>
#include <random>
#include <ct/optcon/optcon.h>
// Using the custom controller from ct_core examples.
#include <CustomController.h>

int main(int argc, char** argv)
{
    // a damped oscillator has two states, position and velocity
    const size_t STATE_DIM   = ct::core::SecondOrderSystem::STATE_DIM;    // = 2
    const size_t CONTROL_DIM = ct::core::SecondOrderSystem::CONTROL_DIM;  // = 1
    const size_t OUTPUT_DIM  = 1;  // We assume we only receive a single observation.

    // create a state
    ct::core::StateVector<STATE_DIM> x;

    // we initialize it at a point with unit deflection and zero velocity
    x(0) = 1.0;
    x(1) = 0.0;

    // create our oscillator
    double w_n = 50;
    std::shared_ptr<ct::core::SecondOrderSystem> oscillator(new ct::core::SecondOrderSystem(w_n));

    // create our controller
    double kp = 10;
    double kd = 1;
    ct::core::ControlVector<CONTROL_DIM> uff;
    uff << 2.0;
    std::shared_ptr<CustomController> controller(new CustomController(uff, kp, kd));

    // assign our controller
    oscillator->setController(controller);

    // create an integrator
    ct::core::Integrator<STATE_DIM> integrator(oscillator, ct::core::IntegrationType::RK4);

    ct::core::StateVectorArray<STATE_DIM, double> states;
    ct::core::tpl::TimeArray<double> times;

    // simulate 1000 steps
    double dt         = 0.001;
    ct::core::Time t0 = 0.0;
    size_t nSteps     = 100;
    integrator.integrate_n_steps(x, t0, nSteps, dt, states, times);

    // Load dt.
    ct::core::OutputStateMatrix<OUTPUT_DIM, STATE_DIM, double> C;
    C << 0.5, 1;  // Measure a combination of position and velocity.
    ct::core::StateMatrix<STATE_DIM, double> Q   = ct::core::StateMatrix<STATE_DIM, double>::Identity();
    ct::core::OutputMatrix<OUTPUT_DIM, double> R = 10 * ct::core::OutputMatrix<OUTPUT_DIM, double>::Identity();

    std::shared_ptr<ct::core::SystemLinearizer<STATE_DIM, CONTROL_DIM, double>> linearizer(
        new ct::core::SystemLinearizer<STATE_DIM, CONTROL_DIM, double>(oscillator));
    ct::core::SensitivityApproximation<STATE_DIM, CONTROL_DIM, STATE_DIM / 2, STATE_DIM / 2, double> sensApprox(
        dt, linearizer);
    ct::optcon::ExtendedKalmanFilter<STATE_DIM, double> ekf(states[0]);
    ct::optcon::StateObserver<OUTPUT_DIM, STATE_DIM, CONTROL_DIM, ct::optcon::ExtendedKalmanFilter<STATE_DIM, double>,
        double>
        stateObserver(oscillator, sensApprox, dt, C, ekf, Q, R);

    std::default_random_engine gen;
    gen.seed(std::chrono::system_clock::now().time_since_epoch().count());
    std::normal_distribution<double> noise(0, 1);

    ct::core::StateVectorArray<STATE_DIM, double> states_est;
    states_est.push_back(states[0]);
    ct::core::StateVector<STATE_DIM, double> xt;
    for (size_t i = 1; i < states.size(); ++i)
    {
        xt = states[i];
        xt[0] += 0.15 * noise(gen);  // Position noise.
        xt[1] += 0.10 * noise(gen);  // Velocity noise.

        stateObserver.predict();
        states_est.push_back(stateObserver.update(C * xt));
    }

    for (size_t i = 0; i < states_est.size(); ++i)
        std::cout << "State\t\tState_est\n"
                  << std::fixed << std::setprecision(6) << states[i][0] << "\t" << states_est[i][0] << std::endl
                  << states[i][1] << "\t" << states_est[i][1] << std::endl
                  << std::endl;

    return 0;
}
