/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace rbd {

/**
 * \brief This is a common interface class for an RBDSystem
 */
template <class RBDDynamics, bool QUAT_INTEGRATION = false>
class RBDSystem
{
public:
    RBDSystem() = default;
    virtual ~RBDSystem() = default;
    virtual RBDDynamics& dynamics() = 0;
    virtual const RBDDynamics& dynamics() const = 0;

private:
};
}  // namespace rbd
}  // namespace ct
