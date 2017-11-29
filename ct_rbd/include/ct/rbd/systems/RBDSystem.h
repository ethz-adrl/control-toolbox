/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#ifndef INCLUDE_CT_RBD_SYSTEMS_RBDSYSTEM_H_
#define INCLUDE_CT_RBD_SYSTEMS_RBDSYSTEM_H_

namespace ct {
namespace rbd {

/**
 * \brief This is a common interface class for an RBDSystem
 */
template <class RBDDynamics, bool QUAT_INTEGRATION = false>
class RBDSystem
{
public:
    RBDSystem() {}
    virtual ~RBDSystem() {}
    virtual RBDDynamics& dynamics() = 0;
    virtual const RBDDynamics& dynamics() const = 0;

private:
};
}
}


#endif /* INCLUDE_CT_RBD_SYSTEMS_RBDSYSTEM_H_ */
