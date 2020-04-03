/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/types/Time.h>
#include <ct/core/types/TypeTraits.h>

namespace ct {
namespace core {

//! type of system
enum SYSTEM_TYPE
{
    GENERAL = 0,  //!< any non-specific system
    SECOND_ORDER  //!< a pure second-order system
};

enum TIME_TYPE : bool
{
    CONTINUOUS_TIME = true,
    DISCRETE_TIME = false
};

//! Interface class for a general system described by an ordinary differential equation (ODE)
/*!
 * Defines the interface for a general system described by an ordinary differential equation (ODE) of the form
 *
 * \f[
 *  \dot{x} = f(x,t)
 * \f]
 *
 * for systems with an input (\f$ \dot{x} = f(x,u,t) \f$) see ControlledSystem.
 *
 * To implement your own system, derive from this class. This ensures you can use other functionality such as
 * an Integrator.
 *
 * @tparam STATE_DIM dimensionality of the state
 * @tparam SCALAR scalar type
 */
template <typename MANIFOLD, bool CONT_T>
class System
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using SCALAR = typename MANIFOLD::Scalar;
    using TANGENT = typename MANIFOLD::Tangent;

    // determine Time_t to be either scalar or int
    using Time_t = typename std::conditional_t<CONT_T, SCALAR, int>;

    //! default constructor
    /*!
	 * Creates a new system given a system type. The system type can help to speed up algorithms that
	 * specialize on the type. If unsure about the type, simply use SYSTEM_TYPE::GENERAL.
	 *
	 * @param type type of system
	 */
    System(const SYSTEM_TYPE& type = SYSTEM_TYPE::GENERAL);

    //! copy constructor
    System(const System& other);

    //! destructor
    virtual ~System();

    //! deep copy
    virtual System* clone() const;

    //! computes the system dynamics
    /*!
	 * evaluates \f$ \dot{x} = f(x,t) \f$ at a given state and time
	 * @param state state to evaluate dynamics at
	 * @param t time to evaluate the dynamics at
	 * @param derivative state derivative
	 */
    virtual void computeDynamics(const MANIFOLD& m, const Time_t& tn, TANGENT& t) = 0;

    //! lift a manifold element to tangent space, default implementation is log w.r.t. epsilon
    virtual TANGENT lift(const MANIFOLD& m);

    //! retract a tangent vector to the corresponding group element, default implementation is w.r.t. epsilon
    virtual MANIFOLD retract(const TANGENT& t);

    //! get the type of system
    /*!
	 * @return system type
	 */
    SYSTEM_TYPE getType() const;
    // default lift specialization for euclidean case
    template <typename T = TANGENT>
    typename std::enable_if<is_euclidean<MANIFOLD>::value, T>::type lift_specialized(const MANIFOLD& m);
    // default lift specialization for manifold-case
    template <typename T = TANGENT>
    typename std::enable_if<!(is_euclidean<MANIFOLD>::value), T>::type lift_specialized(const MANIFOLD& m);
    // default retract specialization for euclidean case
    template <typename T = MANIFOLD>
    typename std::enable_if<is_euclidean<MANIFOLD>::value, T>::type retract_specialized(const TANGENT& t);
    // default retract specialization for manifold case
    template <typename T = MANIFOLD>
    typename std::enable_if<!(is_euclidean<MANIFOLD>::value), T>::type retract_specialized(const TANGENT& t);

protected:
    SYSTEM_TYPE type_;  //!< type of system
};

/**
 * Implementations of default specializations above. 
 * \warning: do not move to *-impl.h file!
 */
template <typename MANIFOLD, bool CONT_T>
template <typename T>
typename std::enable_if<is_euclidean<MANIFOLD>::value, T>::type System<MANIFOLD, CONT_T>::lift_specialized(
    const MANIFOLD& m)
{
    return m;
}
template <typename MANIFOLD, bool CONT_T>
template <typename T>
typename std::enable_if<!(is_euclidean<MANIFOLD>::value), T>::type System<MANIFOLD, CONT_T>::lift_specialized(
    const MANIFOLD& m)
{
    return m.log();
}
template <typename MANIFOLD, bool CONT_T>
template <typename T>
typename std::enable_if<is_euclidean<MANIFOLD>::value, T>::type System<MANIFOLD, CONT_T>::retract_specialized(
    const TANGENT& t)
{
    return t;
}
template <typename MANIFOLD, bool CONT_T>
template <typename T>
typename std::enable_if<!(is_euclidean<MANIFOLD>::value), T>::type System<MANIFOLD, CONT_T>::retract_specialized(
    const TANGENT& t)
{
    return t.exp();
}

}  // namespace core
}  // namespace ct
