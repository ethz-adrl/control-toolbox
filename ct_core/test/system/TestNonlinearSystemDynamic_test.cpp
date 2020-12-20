/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/core/core.h>
#include "TestSystemDynamic.h"

using namespace ct::core;


void RunRandomSystem(const int state_dim, const int control_dim)
{
    std::shared_ptr<TestSystemDynamic> sys(new TestSystemDynamic(state_dim, control_dim, 1.9));

    ControlVectord u(control_dim);
    u.setConstant(1.0);
    sys->setController(std::shared_ptr<ConstantController<EuclideanStateXd, ContinuousTime>>(
        new ConstantController<EuclideanStateXd, ContinuousTime>(u)));

    EuclideanStateXd x;
    x.resize(state_dim);
    x.setConstant(1.0);

    EuclideanStateXd dx;

    sys->computeControlledDynamics(x, 1.0, u, dx);

    std::cout << "dx: " << dx.transpose() << std::endl;

    EuclideanIntegratorXd integrator(sys);

    for (size_t i = 0; i < 10; i++)
    {
        integrator.integrate_n_steps(x, 0.0, 1, 0.01);
        std::cout << "x integr: " << x.transpose() << std::endl;
    }
}

int main()
{
    RunRandomSystem(2, 1);
    RunRandomSystem(3, 2);
}