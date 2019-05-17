/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace rbd {

template <size_t NJOINTS, typename SCALAR>
EndEffector<NJOINTS, SCALAR>::EndEffector() : linkId_(999)
{
}

template <size_t NJOINTS, typename SCALAR>
EndEffector<NJOINTS, SCALAR>::~EndEffector()
{
}

template <size_t NJOINTS, typename SCALAR>
EndEffector<NJOINTS, SCALAR>::EndEffector(const EndEffector& other) : linkId_(other.linkId_)
{
}

template <size_t NJOINTS, typename SCALAR>
const size_t& EndEffector<NJOINTS, SCALAR>::getLinkId()
{
    return linkId_;
}

template <size_t NJOINTS, typename SCALAR>
void EndEffector<NJOINTS, SCALAR>::setLinkId(size_t linkId)
{
    linkId_ = linkId;
}

} /* namespace rbd */
} /* namespace ct */
