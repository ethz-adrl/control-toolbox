/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

/*!
 * \example KalmanDisturbanceFiltering.cpp
 *
 * This example shows how to use the Kalman Filter to estimate the state and the disturbance acting on a simple
 * oscillator. The gist of the example is that the state is augmented with a disturbance and the system dynamics
 * is expanded to include our model of the system with disturbance included.
 *
 */

#include <ct/optcon/optcon.h>

/*!
 * \brief An example controller, in which we artifically add a constant disturbance in order to demonstrate disturbance filtering
 *
 */
//! an example controller class that takes a 2-dimensional state and outputs a 1-dimensional control action
class CustomController : public ct::core::Controller<2, 1>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const size_t STATE_DIM = 2;    // two states
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

    //! copy constructor
    CustomController(const CustomController& other) : uff_(other.uff_), kp_(other.kp_), kd_(other.kd_), d_(other.d_) {}
    //! clone method, needs to be implemented, overrides ct::core::Controller::clone()
    CustomController* clone() const override { return new CustomController(*this); }
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
 * An example implementation of a disturbed system with pure input disturbance
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
        : ct::optcon::DisturbedSystem<STATE_DIM, DIST_DIM, CONTROL_DIM, SCALAR>(other.system_->getController()),
          system_(other.system_)
    {
    }

    //! deep cloning
    CustomDisturbedSystem* clone() const override { return new CustomDisturbedSystem(*this); }
    /*!
     * Override the computeControlledDynamics with a custom update. Since only input disturbance is present, we
     * the dynamics w.r.t. state is calculated by correcting the control action with our disturbance estimate. Since
     * the disturbance is assumed constant, the derivative w.r.t the disturbance is zero.
     */
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
    const size_t STATE_DIM = ct::core::SecondOrderSystem::STATE_DIM;      // = 2
    const size_t CONTROL_DIM = ct::core::SecondOrderSystem::CONTROL_DIM;  // = 1
    const size_t OUTPUT_DIM = 2;
    const size_t DIST_DIM = CONTROL_DIM;  // Here we only use input disturbance.

    // create an initial state: we initialize it at a point with unit deflection and zero velocity
    ct::core::StateVector<STATE_DIM> x;
    x(0) = 1.0;
    x(1) = 0.0;

    // create an oscillator
    double w_n = 50;
    std::shared_ptr<ct::core::SecondOrderSystem> oscillator(new ct::core::SecondOrderSystem(w_n));

    // create our controller: a PD controller which also "simulates" a constant disturbance
    double kp = 10;
    double kd = 1;
    double disturbance = 1;
    ct::core::ControlVector<CONTROL_DIM> uff;
    uff << 2.0;
    std::shared_ptr<CustomController> disturbed_controller(new CustomController(uff, kp, kd, disturbance));

    // assign our controller to generate the disturbed data
    oscillator->setController(disturbed_controller);

    // create an integrator for "simulating" the measured data
    ct::core::Integrator<STATE_DIM> integrator(oscillator, ct::core::IntegrationType::RK4);

    ct::core::StateVectorArray<STATE_DIM> states;
    ct::core::tpl::TimeArray<double> times;

    // simulate 1000 steps
    double dt = 0.001;
    ct::core::Time t0 = 0.0;
    size_t nSteps = 1000;
    integrator.integrate_n_steps(x, t0, nSteps, dt, states, times);

    std::shared_ptr<CustomController> controller(new CustomController(uff, kp, kd));

    // assign the controller that we "assume" we are using
    oscillator->setController(controller);
    std::shared_ptr<CustomDisturbedSystem<STATE_DIM, DIST_DIM, CONTROL_DIM>> customdisturbedSystem(
        new CustomDisturbedSystem<STATE_DIM, DIST_DIM, CONTROL_DIM>(oscillator));

    // Observation matrix for the state
    ct::core::OutputStateMatrix<OUTPUT_DIM, STATE_DIM> C;
    C.setIdentity();

    // Observation matrix for the disturbance (assuming we don't observe the disturbance at all)
    ct::core::OutputStateMatrix<OUTPUT_DIM, DIST_DIM> Cd;
    Cd.setZero();

    // Total observation matrix, state and disturbance combined
    ct::core::OutputStateMatrix<OUTPUT_DIM, STATE_DIM + DIST_DIM, double> Caug;
    Caug << C, Cd;

    // filter weighting matrices Q and R
    ct::core::StateMatrix<STATE_DIM + DIST_DIM> Qaug = ct::core::StateMatrix<STATE_DIM + DIST_DIM>::Identity();
    Qaug(2, 2) *= 100;
    ct::core::OutputMatrix<OUTPUT_DIM> R = ct::core::OutputMatrix<OUTPUT_DIM>::Identity();

    // create a sensitivity approximator to obtain discrete-time A and B matrices
    std::shared_ptr<ct::core::SystemLinearizer<STATE_DIM + DIST_DIM, CONTROL_DIM>> linearizer(
        new ct::core::SystemLinearizer<STATE_DIM + DIST_DIM, CONTROL_DIM>(customdisturbedSystem));
    ct::core::SensitivityApproximation<STATE_DIM + DIST_DIM, CONTROL_DIM> sensApprox(dt, linearizer);

    ct::core::StateVector<STATE_DIM + DIST_DIM> x0aug;
    x0aug << states[0], 0;
    ct::optcon::ExtendedKalmanFilter<STATE_DIM + DIST_DIM> ekf(x0aug);
    ct::optcon::DisturbanceObserver<OUTPUT_DIM, STATE_DIM, DIST_DIM, CONTROL_DIM,
        ct::optcon::ExtendedKalmanFilter<STATE_DIM + DIST_DIM>>
        disturbanceObserver(customdisturbedSystem, sensApprox, dt, Caug, ekf, Qaug, R);

    ct::core::GaussianNoise noise(0, 0.0005);

    ct::core::StateVectorArray<STATE_DIM + DIST_DIM> states_est;
    states_est.push_back(x0aug);
    ct::core::StateVector<STATE_DIM> xt;
    Eigen::Matrix<double, DIST_DIM, 1> d = disturbance * Eigen::Matrix<double, DIST_DIM, 1>::Ones();
    for (size_t i = 1; i < states.size(); ++i)
    {
        // compute an observation
        xt = states[i];
        noise.noisify(xt[0]);  // Position noise.
        ct::core::OutputVector<OUTPUT_DIM> y = C * xt + Cd * d;

        // Kalman filter prediction step
        disturbanceObserver.predict();

        // Kalman filter estimation step (state + disturbance)
        ct::core::StateVector<STATE_DIM + DIST_DIM> x_est = disturbanceObserver.update(y);
        states_est.push_back(x_est);

        // print result
        std::cout << "State\t\tState_est\tdisturbance_est\n"
                  << std::fixed << std::setprecision(6) << states[i][0] << "\t" << states_est[i][0] << "\t"
                  << states_est[i][2] << std::endl
                  << states[i][1] << "\t" << states_est[i][1] << std::endl
                  << std::endl;
    }

    return 0;
}
