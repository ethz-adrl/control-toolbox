/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace models {
namespace InvertedPendulum {

template <typename SCALAR = double>
const typename ct::rbd::tpl::JointState<1, SCALAR>::Position& jointLowerLimit()
{
    static typename ct::rbd::tpl::JointState<1, SCALAR>::Position jointLowerLimit;
    jointLowerLimit << -3.1416;

    return jointLowerLimit;
}

template <typename SCALAR = double>
const typename ct::rbd::tpl::JointState<1, SCALAR>::Position& jointUpperLimit()
{
    static typename ct::rbd::tpl::JointState<1, SCALAR>::Position jointUpperLimit;
    jointUpperLimit << 3.1415;

    return jointUpperLimit;
}

template <typename SCALAR = double>
const typename ct::rbd::tpl::JointState<1, SCALAR>::Velocity& jointVelocityLimit()
{
    static typename ct::rbd::tpl::JointState<1, SCALAR>::Velocity jointVelocityLimit;
    jointVelocityLimit << 2.0;

    return jointVelocityLimit;
}

}
}
}
