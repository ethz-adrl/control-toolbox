/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {

template <typename MANIFOLD, typename SCALAR>
System<MANIFOLD, SCALAR>::System(const SYSTEM_TYPE& type) : type_(type)
{
}

template <typename MANIFOLD, typename SCALAR>
System<MANIFOLD, SCALAR>::~System()
{
}

template <typename MANIFOLD, typename SCALAR>
System<MANIFOLD, SCALAR>::System(const System& other) : type_(other.type_)
{
}

template <typename MANIFOLD, typename SCALAR>
System<MANIFOLD, SCALAR>* System<MANIFOLD, SCALAR>::clone() const
{
    throw std::runtime_error("clone not implemented");
};

template <typename MANIFOLD, typename SCALAR>
auto System<MANIFOLD, SCALAR>::getType() const -> SYSTEM_TYPE
{
    return type_;
}

template <typename MANIFOLD, typename SCALAR>
bool System<MANIFOLD, SCALAR>::isSymplectic() const
{
    return false;
}

}  // namespace core
}  // namespace ct
