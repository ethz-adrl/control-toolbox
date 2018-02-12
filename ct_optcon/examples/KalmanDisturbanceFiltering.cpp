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
        const double& kd,                                              // D gain
        const double& disturbance = 0                                  // disturbance
        )
        : uff_(uff), kp_(kp), kd_(kd), d_(disturbance)
    {
    }

    //! destructor
    ~CustomController() {}
    //! copy constructor
    CustomController(const CustomController& other) : uff_(other.uff_), kp_(other.kp_), kd_(other.kd_), d_(other.d_) {}
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
        controlAction = uff_;                                      // apply feedforward control
        controlAction(0) += d_ - kp_ * state(0) - kd_ * state(1);  // add feedback control and disturbance
    }

private:
    ct::core::ControlVector<CONTROL_DIM> uff_;  //! feedforward control action
    double kp_;                                 //! P gain
    double kd_;                                 //! D gain
    double d_;                                  //! disturbance
};

/*!
 * An implementation of a disturbed system with input disturbance
 */
template <size_t STATE_DIM, size_t DIST_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class CustomDisturbedSystem : public ct::optcon::DisturbedSystem<STATE_DIM, DIST_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CustomDisturbedSystem(std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>> sys)
        : ct::optcon::DisturbedSystem<STATE_DIM, DIST_DIM, CONTROL_DIM, SCALAR>(sys->getController()), system_(sys)
    {
    }

    CustomDisturbedSystem(const CustomDisturbedSystem& other)
        : ct::optcon::DisturbedSystem<STATE_DIM, DIST_DIM, CONTROL_DIM, SCALAR>(other.system_->getController()),
          system_(other.system_)
    {
    }

    CustomDisturbedSystem* clone() const override { return new CustomDisturbedSystem(*this); }
    //! override the computeControlledDynamics with a custom update
    void computeControlledDynamics(const ct::core::StateVector<STATE_DIM + DIST_DIM, SCALAR>& state,
        const SCALAR& t,
        const ct::core::ControlVector<CONTROL_DIM, SCALAR>& control,
        ct::core::StateVector<STATE_DIM + DIST_DIM, SCALAR>& derivative) override
    {
        derivative.setZero();
        ct::core::StateVector<STATE_DIM, SCALAR> tempDerivative;
        system_->computeControlledDynamics(
            state.head(STATE_DIM), t, control.toImplementation() + state.tail(DIST_DIM), tempDerivative);
        derivative.head(STATE_DIM) = tempDerivative;
    }

private:
    std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>> system_;
};

int main(int argc, char** argv)
{
    // a damped oscillator has two states, position and velocity
    const size_t STATE_DIM   = ct::core::SecondOrderSystem::STATE_DIM;    // = 2
    const size_t CONTROL_DIM = ct::core::SecondOrderSystem::CONTROL_DIM;  // = 1
    const size_t OBS_DIM     = 2;            // We assume we only receive a single observation.
    const size_t DIST_DIM    = CONTROL_DIM;  // Here we only use input disturbance.

    // create a state
    ct::core::StateVector<STATE_DIM> x;

    // we initialize it at a point with unit deflection and zero velocity
    x(0) = 1.0;
    x(1) = 0.0;

    // create our oscillator
    double w_n = 50;
    std::shared_ptr<ct::core::SecondOrderSystem> oscillator(new ct::core::SecondOrderSystem(w_n));
    // create our controller
    double kp          = 10;
    double kd          = 1;
    double disturbance = 1;
    ct::core::ControlVector<CONTROL_DIM> uff;
    uff << 2.0;
    std::shared_ptr<CustomController> disturbed_controller(new CustomController(uff, kp, kd, disturbance));

    // assign our controller to generate the disturbed data
    oscillator->setController(disturbed_controller);

    // create an integrator
    ct::core::Integrator<STATE_DIM> integrator(oscillator, ct::core::IntegrationType::RK4);

    ct::core::StateVectorArray<STATE_DIM, double> states;
    ct::core::tpl::TimeArray<double> times;

    // simulate 1000 steps
    double dt         = 0.001;
    ct::core::Time t0 = 0.0;
    size_t nSteps     = 1000;
    integrator.integrate_n_steps(x, t0, nSteps, dt, states, times);

    std::shared_ptr<CustomController> controller(new CustomController(uff, kp, kd));

    // assign the controller that we "assume" we are using
    oscillator->setController(controller);
    std::shared_ptr<CustomDisturbedSystem<STATE_DIM, DIST_DIM, CONTROL_DIM, double>> customdisturbedSystem(
        new CustomDisturbedSystem<STATE_DIM, DIST_DIM, CONTROL_DIM, double>(oscillator));

    // Observation matrix.
    ct::core::OutputStateMatrix<OBS_DIM, STATE_DIM, double> C;
    C.setIdentity();

    ct::core::OutputStateMatrix<OBS_DIM, DIST_DIM, double> Cd;
    Cd.setZero();
    // Augmented C matrix assuming we don't observe the disturbance.
    ct::core::OutputStateMatrix<OBS_DIM, STATE_DIM + DIST_DIM, double> Caug;
    Caug << C, Cd;
    ct::core::StateMatrix<STATE_DIM + DIST_DIM, double> Qaug =
        ct::core::StateMatrix<STATE_DIM + DIST_DIM, double>::Identity();
    Qaug(2, 2) *= 100;
    ct::core::OutputMatrix<OBS_DIM, double> R = ct::core::OutputMatrix<OBS_DIM, double>::Identity();

    std::shared_ptr<ct::core::SystemLinearizer<STATE_DIM + DIST_DIM, CONTROL_DIM, double>> linearizer(
        new ct::core::SystemLinearizer<STATE_DIM + DIST_DIM, CONTROL_DIM, double>(customdisturbedSystem));
    ct::core::SensitivityApproximation<STATE_DIM + DIST_DIM, CONTROL_DIM> sensApprox(dt, linearizer);

    ct::core::StateVector<STATE_DIM + DIST_DIM> x0aug;
    x0aug << states[0], 0;
    ct::optcon::ExtendedKalmanFilter<STATE_DIM + DIST_DIM, double> ekf(x0aug);
    ct::optcon::DisturbanceObserver<OBS_DIM, STATE_DIM, DIST_DIM, CONTROL_DIM,
        ct::optcon::ExtendedKalmanFilter<STATE_DIM + DIST_DIM, double>, double>
        disturbanceObserver(customdisturbedSystem, sensApprox, dt, Caug, ekf, Qaug, R);

    std::default_random_engine gen;
    gen.seed(std::chrono::system_clock::now().time_since_epoch().count());
    std::normal_distribution<double> noise(0, 1);

    ct::core::StateVectorArray<STATE_DIM + DIST_DIM, double> states_est;
    states_est.push_back(x0aug);
    ct::core::StateVector<STATE_DIM, double> xt;
    Eigen::Matrix<double, DIST_DIM, 1> d = disturbance * Eigen::Matrix<double, DIST_DIM, 1>::Ones();
    for (size_t i = 1; i < states.size(); ++i)
    {
        xt = states[i];
        xt[0] += 0.0005 * noise(gen);  // Position noise.

        disturbanceObserver.predict();
        states_est.push_back(disturbanceObserver.update(C * xt + Cd * d));
        std::cout << "State\t\tState_est\n"
                  << std::fixed << std::setprecision(6) << states[i][0] << "\t" << states_est[i][0] << std::endl
                  << states[i][1] << "\t" << states_est[i][1] << std::endl
                  << "\t\t" << states_est[i][2] << std::endl
                  << std::endl;
    }


    return 0;
}
