/***********************************************************************************
Copyright (c) 2017, Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo,
Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be used
      to endorse or promote products derived from this software without specific
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/

#ifndef CT_SYSTEM_H_
#define CT_SYSTEM_H_

#include <ct/core/types/Time.h>
#include <ct/core/types/StateVector.h>

namespace ct {
namespace core {

//! type of system
enum SYSTEM_TYPE {
    GENERAL = 0, //!< any non-specific system
    SECOND_ORDER //!< a pure second-order system
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
template <size_t STATE_DIM, typename SCALAR = double>
class System
{
public:
	typedef SCALAR S; //!< the scalar type

	//! default constructor
	/*!
	 * Creates a new system given a system type. The system type can help to speed up algorithms that
	 * specialize on the type. If unsure about the type, simply use SYSTEM_TYPE::GENERAL.
	 *
	 * @param type type of system
	 */
	System(const SYSTEM_TYPE& type = SYSTEM_TYPE::GENERAL) :
	    type_(type)
	{}

	//! copy constructor
	System(const System& other) :
		type_(other.type_)
	{}

	//! destructor
	virtual ~System() {}

	//! deep copy
	virtual System* clone() const { throw std::runtime_error("clone not implemented"); } ;

	//! computes the system dynamics
	/*!
	 * evaluates \f$ \dot{x} = f(x,t) \f$ at a given state and time
	 * @param state state to evaluate dynamics at
	 * @param t time to evaluate the dynamics at
	 * @param derivative state derivative
	 */
	virtual void computeDynamics(
			const StateVector<STATE_DIM, SCALAR>& state,
			const SCALAR& t,
			StateVector<STATE_DIM, SCALAR>& derivative) = 0;

	//! get the type of system
	/*!
	 * @return system type
	 */
	SYSTEM_TYPE getType() const { return type_; }

protected:

	SYSTEM_TYPE type_; //!< type of system
};

} // core
} // ct


#endif /* SYSTEMBASE_H_ */
