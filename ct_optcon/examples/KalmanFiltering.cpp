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
    static const size_t STATE_DIM   = 2;  // two states
    static const size_t CONTROL_DIM = 1;  // one control action

    //! default constructor
    CustomController(const ct::core::ControlVector<CONTROL_DIM>& uff,  // feedforward control
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
    void computeControl(const ct::core::StateVector<STATE_DIM>& state,
        const double& t,
        ct::core::ControlVector<CONTROL_DIM>& controlAction) override
    {
        controlAction = uff_;                                 // apply feedforward control
        controlAction(0) -= kp_ * state(0) + kd_ * state(1);  // add feedback control
    }

private:
    ct::core::ControlVector<CONTROL_DIM> uff_;  //! feedforward control action
    double kp_;                                 //! P gain
    double kd_;                                 //! D gain
};

int main(int argc, char** argv)
{
    // a damped oscillator has two states, position and velocity
    const size_t STATE_DIM   = ct::core::SecondOrderSystem::STATE_DIM;    // = 2
    const size_t CONTROL_DIM = ct::core::SecondOrderSystem::CONTROL_DIM;  // = 1
    const size_t OBS_DIM     = 1;  // We assume we only receive a single observation.

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

    ct::core::ControlVectorArray<CONTROL_DIM, double> controls(states.size() - 1);
    for (size_t i = 0; i < states.size() - 1; ++i)
        controller->computeControl(states[i], times[i], controls[i]);

    // Load dt.
    ct::core::OutputStateMatrix<OBS_DIM, STATE_DIM, double> C;
    C << 0.5, 1;  // Measure position.
    ct::core::StateMatrix<STATE_DIM, double> Q = ct::core::StateMatrix<STATE_DIM, double>::Identity();
    ct::core::OutputMatrix<OBS_DIM, double> R  = 10 * ct::core::OutputMatrix<OBS_DIM, double>::Identity();

    std::shared_ptr<ct::core::SystemLinearizer<STATE_DIM, CONTROL_DIM, double>> linearizer(
        new ct::core::SystemLinearizer<STATE_DIM, CONTROL_DIM, double>(oscillator));
    ct::core::SensitivityApproximation<STATE_DIM, CONTROL_DIM, STATE_DIM / 2, STATE_DIM / 2, double> sensApprox(
        dt, linearizer);
    ct::optcon::ExtendedKalmanFilter<STATE_DIM, double> ekf(states[0]);
    ct::optcon::StateObserver<OBS_DIM, STATE_DIM, CONTROL_DIM, ct::optcon::ExtendedKalmanFilter<STATE_DIM, double>,
        double>
        stateObserver(oscillator, sensApprox, dt, C, ekf, Q, R);

    std::default_random_engine gen;
    gen.seed(std::chrono::system_clock::now().time_since_epoch().count());
    std::normal_distribution<double> noise(0, 1);

    ct::core::StateVectorArray<STATE_DIM, double> states_est;
    ct::core::StateVector<STATE_DIM, double> xt;
    for (size_t i = 0; i < controls.size(); ++i)
    {
        xt = states[i];
        xt[0] += 0.15 * noise(gen);  // Position noise.
        xt[1] += 0.10 * noise(gen);  // Velocity noise.

        stateObserver.predict();
        states_est.push_back(stateObserver.update(C * xt));
    }

    for (size_t i = 0; i < states_est.size(); ++i)
        std::cerr << "State\t\tState_est\n"
                  << std::fixed << std::setprecision(6)
                  << states[i][0] << "\t" << states_est[i][0] << std::endl
                  << states[i][1] << "\t" << states_est[i][1] << std::endl
                  << std::endl;

    return 0;
}
