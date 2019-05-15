/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/rbd/state/JointState.h>
#include <ct/rbd/state/RigidBodyPose.h>

namespace ct {
namespace rbd {

template <size_t NJOINTS, typename SCALAR = double>
class EndEffector
{
public:
    typedef Eigen::Matrix<SCALAR, 6, NJOINTS> jacobian_t;
    typedef typename JointState<NJOINTS>::Position joint_position_t;

    EndEffector();

    virtual ~EndEffector();

    EndEffector(const EndEffector& other);

    /**
	 * \brief Return the ID of the link to which the end-effector is rigidly attached to
	 * @return Link ID
	 */
    const size_t& getLinkId();

    /**
	 * \brief *DO NOT USE*. Set the link id on which an endeffector is on
	 * @param linkId LinkId to be set
	 */
    void setLinkId(size_t linkId);  // we should not have this public

private:
    // id of the link that the endeffector is on
    size_t linkId_;
};

} /* namespace rbd */
} /* namespace ct */
