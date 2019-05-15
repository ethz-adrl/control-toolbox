/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

namespace ct {
namespace optcon {
namespace example {

using namespace ct::core;
using namespace ct::optcon;

using std::shared_ptr;

const size_t state_dim = 1;
const size_t control_dim = 1;


//! Dynamics class for the GNMS unit test, slightly nonlinear dynamics
class Dynamics : public ControlledSystem<state_dim, control_dim>
{
public:
    Dynamics() : ControlledSystem<state_dim, control_dim>(SYSTEM_TYPE::SECOND_ORDER) {}
    void computeControlledDynamics(const StateVector<state_dim>& state,
        const Time& t,
        const ControlVector<control_dim>& control,
        StateVector<state_dim>& derivative) override
    {
        derivative(0) = (1.0 + state(0)) * state(0) + control(0);
    }

    Dynamics* clone() const override { return new Dynamics(); };
};


//! Linear system class for the GNMS unit test
class LinearizedSystem : public LinearSystem<state_dim, control_dim>
{
public:
    state_matrix_t A_;
    state_control_matrix_t B_;


    const state_matrix_t& getDerivativeState(const StateVector<state_dim>& x,
        const ControlVector<control_dim>& u,
        const double t = 0.0) override
    {
        A_ << 1 + 2 * x(0);
        return A_;
    }

    const state_control_matrix_t& getDerivativeControl(const StateVector<state_dim>& x,
        const ControlVector<control_dim>& u,
        const double t = 0.0) override
    {
        B_ << 1;
        return B_;
    }

    LinearizedSystem* clone() const override { return new LinearizedSystem(); }
};
}
}
}
