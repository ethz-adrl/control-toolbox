/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/core/core.h>
#include "TestSystemSE2.h"

using namespace ct::core;

int main()
{
    const size_t control_dim = 3;

    ControlVectord u(control_dim);
    u.setConstant(0.0);
    std::shared_ptr<ConstantController<ct::core::SE2d, ContinuousTime>> controller(
        new ConstantController<ct::core::SE2d, ContinuousTime>(u));


    ct::core::SE2d m = ct::core::SE2d::NeutralElement();
    ct::core::SE2d m_ref = ct::core::SE2d::NeutralElement();
    m_ref.coeffs() << 1.0, 0.0, 0.0, 1.0;

    std::shared_ptr<TestSystemSE2> sys(new TestSystemSE2(1.0, controller, m_ref));

    Integrator<SE2d> integrator(sys);

    std::cout << m << std::endl;
    for (size_t i = 0; i < 10; i++)
    {
        integrator.integrate_n_steps(m, 0.0, 1, 0.5);
        std::cout << m << std::endl;
    }

    // should end up at m_ref, 1, 0, 0, 1.

    return 1;
}