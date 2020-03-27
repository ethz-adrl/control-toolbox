/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {

template <typename MANIFOLD, bool CONT_T>
System<MANIFOLD, CONT_T>::System(const SYSTEM_TYPE& type) : type_(type)
{
}

template <typename MANIFOLD, bool CONT_T>
System<MANIFOLD, CONT_T>::~System()
{
}

template <typename MANIFOLD, bool CONT_T>
System<MANIFOLD, CONT_T>::System(const System& other) : type_(other.type_)
{
}

template <typename MANIFOLD, bool CONT_T>
typename MANIFOLD::Tangent System<MANIFOLD, CONT_T>::lift(const MANIFOLD& m)
{
    return lift_specialized(m);
}

template <typename MANIFOLD, bool CONT_T>
MANIFOLD System<MANIFOLD, CONT_T>::retract(const typename MANIFOLD::Tangent& t)
{
    return retract_specialized(t);
}

template <typename MANIFOLD, bool CONT_T>
System<MANIFOLD, CONT_T>* System<MANIFOLD, CONT_T>::clone() const
{
    throw std::runtime_error("clone not implemented");
};

template <typename MANIFOLD, bool CONT_T>
auto System<MANIFOLD, CONT_T>::getType() const -> SYSTEM_TYPE
{
    return type_;
}

template <typename MANIFOLD, bool CONT_T>
auto System<MANIFOLD, CONT_T>::getTimeType() const -> const TIME_TYPE
{
    return CONT_T;
}

}  // namespace core
}  // namespace ct
