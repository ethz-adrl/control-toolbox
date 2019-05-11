/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace rbd {
namespace TestIrb4600 {

template <typename SCALAR = double>
const typename ct::rbd::JointState<6, SCALAR>::Position& jointLowerLimit()
{
    static typename ct::rbd::JointState<6, SCALAR>::Position jointLowerLimit;
    jointLowerLimit << -3.1416, -1.5708, -3.14159, -6.98132, -2.18166, -6.98132;

    return jointLowerLimit;
}

template <typename SCALAR = double>
const typename ct::rbd::JointState<6, SCALAR>::Position& jointUpperLimit()
{
    static typename ct::rbd::JointState<6, SCALAR>::Position jointUpperLimit;
    jointUpperLimit << 3.1416, 2.61799, 1.309, 6.98132, 2.0944, 6.98132;

    return jointUpperLimit;
}

template <typename SCALAR = double>
const typename ct::rbd::JointState<6, SCALAR>::Velocity& jointVelocityLimit()
{
    static typename ct::rbd::JointState<6, SCALAR>::Velocity jointVelocityLimit;
    jointVelocityLimit << 12.0, 12.0, 12.0, 12.0, 12.0, 12.0;

    return jointVelocityLimit;
}
}  // TestIrb4600
}  // rbd
}  // ct
