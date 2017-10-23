/*!
 *
 * \example DampedOscillator.cpp
 */

#include <ct/core/core.h>

int main(int argc, char* argv)
{
    // a damped oscillator has two states, position and velocity
    const size_t state_dim = ct::core::SecondOrderSystem::STATE_DIM;  // = 2

    // create a state
    ct::core::StateVector<state_dim> x;

    // we initialize it at 0
    x.setZero();

    // create our oscillator
    double w_n = 10;
    std::shared_ptr<ct::core::SecondOrderSystem> oscillator(new ct::core::SecondOrderSystem(w_n));

    // create an integrator
    ct::core::Integrator<state_dim> integrator(oscillator);

    // simulate 1000 steps
    double dt = 0.001;
    ct::core::Time t0 = 0.0;
    size_t nSteps = 1000;
    integrator.integrate_n_steps(x, t0, nSteps, dt);

    // print the new state
    std::cout << "state after integration: " << x.transpose() << std::endl;

    return 1;
}
