/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <memory>

namespace ct {
namespace rbd {

template <class RBD>
class FloatingBaseTransforms
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FloatingBaseTransforms(std::shared_ptr<RBD> rbdContainer) : rbdContainer_(rbdContainer) {}
    virtual ~FloatingBaseTransforms(){};


private:
    std::shared_ptr<RBD> rbdContainer_;
};

} /* namespace rbd */
} /* namespace ct */
