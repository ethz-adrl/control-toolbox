/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/rbd/state/RBDState.h>

namespace ct {
namespace rbd {

/**
 * \ingroup OS
 * @Brief This class is an interface class for operational space coordinate.
 */
template <size_t NUM_OUTPUTS, size_t NUM_JOINTS>
class CoordinateBase
{
public:
    typedef std::shared_ptr<CoordinateBase<NUM_OUTPUTS, NUM_JOINTS>> ptr;
    typedef RBDState<NUM_JOINTS> state_t;
    typedef Eigen::Matrix<double, NUM_OUTPUTS, 1> coordinate_t;

    CoordinateBase() {}
    virtual ~CoordinateBase() {}
    virtual coordinate_t getCoordinate(const state_t& state) = 0;

private:
};

}  // namespace rbd
}  // namespace ct
