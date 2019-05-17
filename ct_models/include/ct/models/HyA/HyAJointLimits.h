/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace models {
namespace HyA {

template <typename SCALAR = double>
const typename ct::rbd::JointState<6, SCALAR>::Position& jointLowerLimit()
{
    static typename ct::rbd::JointState<6, SCALAR>::Position jointLowerLimit;
    jointLowerLimit << -3.1416, -0.7679, -1.6406, 0, -2.0944, -0.5236;

    return jointLowerLimit;
}

template <typename SCALAR = double>
const typename ct::rbd::JointState<6, SCALAR>::Position& jointUpperLimit()
{
    static typename ct::rbd::JointState<6, SCALAR>::Position jointUpperLimit;
    jointUpperLimit << 0.5236, 0.8552, 0.0698, 2.2689, 1.5708, 1.5708;

    return jointUpperLimit;
}

template <typename SCALAR = double>
const typename ct::rbd::JointState<6, SCALAR>::Velocity& jointVelocityLimit()
{
    static typename ct::rbd::JointState<6, SCALAR>::Velocity jointVelocityLimit;
    jointVelocityLimit << 12.0, 12.0, 12.0, 12.0, 12.0, 12.0;

    return jointVelocityLimit;
}
}
}
}
