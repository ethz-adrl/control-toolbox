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

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/
#pragma once

namespace ct {
namespace core {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
class DiscreteSystem
{
public:
	//! constructor
	DiscreteSystem(const SYSTEM_TYPE& type = GENERAL) : type_(type), isSymplectic_(false) {}
	//! desctructor
	virtual ~DiscreteSystem() {}
	//! deep copy
	virtual DiscreteSystem* clone() const { throw std::runtime_error("clone not implemented"); };
	//! propagates the system dynamics forward by one step
	/*!
	 * evaluates \f$ x_{n+1} = f(x_n, n) \f$ at a given state and index
	 * @param state start state to propagate from
	 * @param n time index to propagate the dynamics at
	 * @param stateNext propagated state
	 */
	virtual void propagateDynamics(const StateVector<STATE_DIM, SCALAR>& state,
		const int& n,
		StateVector<STATE_DIM, SCALAR>& stateNext) = 0;

	//! get the type of system
	/*!
	 * @return system type
	 */
	SYSTEM_TYPE getType() const { return type_; }
	/**
	 * @brief      Determines if the system is in symplectic form .
	 *
	 * @return     True if symplectic, False otherwise.
	 */
	bool isSymplectic() const { return isSymplectic_; }
protected:
	SYSTEM_TYPE type_;  //!< type of system

	bool isSymplectic_;
};
}
}
