/*!
 *
 * \example MasspointIntegration.cpp
 */

#include <ct/core/core.h>
#include <ct/core/examples/Masspoint.h>

int main(int argc, char** argv)
{
    // a damped oscillator has two states, position and velocity
    const size_t state_dim = Masspoint::state_dim;  // = 2

    // create a state
    using State = Masspoint::State;
    State x;

    // we initialize it at 1
    x << 1, 0;

    // create our mass point instance
    double mass = 1.0;
    double d = 0.01;
    std::shared_ptr<Masspoint> masspoint(new Masspoint(mass, d));

    // create an integrator
    ct::core::Integrator<State> integrator(masspoint);

    // simulate 1000 steps
    double dt = 0.001;
    ct::core::Time t0 = 0.0;
    size_t nSteps = 1000;
    integrator.integrate_n_steps(x, t0, nSteps, dt);

    // print the new state
    std::cout << "state after integration: " << x.transpose() << std::endl;

    return 0;
}
