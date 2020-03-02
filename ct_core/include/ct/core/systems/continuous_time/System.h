/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/types/Time.h>

namespace ct {
namespace core {

//! type of system
enum SYSTEM_TYPE
{
    GENERAL = 0,  //!< any non-specific system
    SECOND_ORDER  //!< a pure second-order system
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
template <typename MANIFOLD, typename SCALAR = typename MANIFOLD::Scalar>
class System
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using TANGENT = typename MANIFOLD::Tangent;
    typedef SCALAR S;       //!< the scalar type
    typedef SCALAR Time_t;  //!< the type of the time variable

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
    virtual void computeDynamics(const MANIFOLD& m, const Time_t& t, TANGENT& derivative) = 0;

    //! get the type of system
    /*!
	 * @return system type
	 */
    SYSTEM_TYPE getType() const;
    /**
	 * @brief      Determines if the system is in symplectic form.
	 *
	 * @return     True if symplectic, False otherwise.
	 */
    virtual bool isSymplectic() const;

protected:
    SYSTEM_TYPE type_;  //!< type of system
};

}  // namespace core
}  // namespace ct
