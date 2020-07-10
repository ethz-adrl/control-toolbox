/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/core/core.h>
#include "TestNonlinearSystemDynamic.h"

using namespace ct::core;

int main()
{
    std::shared_ptr<TestNonlinearSystemDynamic> sys(new TestNonlinearSystemDynamic(1.9));

    EuclideanStateXd x;
    x.resize(2);
    x << 1.0, 1.0;

    ControlVector<-1> u;
    u.resize(1);
    u << 1.0;

    EuclideanStateXd dx;

    sys->computeControlledDynamics(x, 1.0, u, dx);

    std::cout << "dx: " << dx.transpose() << std::endl;

    EuclideanIntegratorXd integrator(sys);

    integrator.integrate_n_steps(x, 0.0, 1, 0.001);

    std::cout << "x integr: " << x.transpose() << std::endl;
}