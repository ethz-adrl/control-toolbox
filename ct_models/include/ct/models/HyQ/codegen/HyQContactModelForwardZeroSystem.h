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

#ifndef INCLUDE_CT_MODELS_HYQ_CONTACTMODEL_FORWARDZERO_SYSTEM_H_
#define INCLUDE_CT_MODELS_HYQ_CONTACTMODEL_FORWARDZERO_SYSTEM_H_


#include <ct/core/systems/ControlledSystem.h>
#include <ct/rbd/rbd.h>

#include <ct/rbd/state/RigidBodyPose.h>
#include <ct/rbd/physics/EEContactModel.h>

#include "HyQForwardZero.h"

namespace ct {
namespace models {
namespace HyQ {

/**
 * \brief A floating base rigid body system that uses forward dynamics. The input vector
 * is assumed to consist of joint torques and end-effector forces expressed in the world.
 */
class HyQContactModelForwardZeroSystem : public core::SymplecticSystem<18, 18, 12>
{
public:
	const static size_t STATE_DIM = 36;
	const static size_t CONTROL_DIM = 12;

	typedef core::StateVector<STATE_DIM> StateVector;
	typedef core::ControlVector<CONTROL_DIM> ControlVector;

	typedef core::SymplecticSystem<18, 18, CONTROL_DIM> Base;

	HyQContactModelForwardZeroSystem() :
		Base()
	{};

	HyQContactModelForwardZeroSystem(const HyQContactModelForwardZeroSystem& other) :
		Base(other),
		hyqForwardZero_(other.hyqForwardZero_)
	{};

	virtual ~HyQContactModelForwardZeroSystem() {};

	virtual HyQContactModelForwardZeroSystem* clone() const override{
		return new HyQContactModelForwardZeroSystem(*this);
	}

	virtual void computePdot(
			const StateVector& x,
			const core::StateVector<18>& v,
			const ControlVector& control,
			core::StateVector<18>& pDot
		) override
	{
		StateVector xLocal = x;
		xLocal.tail(18) = v;
		ct::rbd::RBDState<12> rbdCached = RBDStateFromVector(xLocal);
		ct::rbd::RBDAcceleration<12> xd;
		pDot = toStateDerivative(xd, rbdCached).head(18);
	}

	virtual void computeVdot(
		const StateVector& x,
		const core::StateVector<18>& p,
		const ControlVector& control,
		core::StateVector<18>& vDot
	) override
	{
		StateVector xLocal = x;
		xLocal.head(18) = p;
		Eigen::Matrix<double, STATE_DIM + CONTROL_DIM + 1, 1> xut;
		xut << xLocal, control, 0.0;
		vDot = hyqForwardZero_(xut).tail(18);
	}


private:
	HyQForwardZero hyqForwardZero_;

	StateVector toStateDerivative(
			const ct::rbd::RBDAcceleration<12>& acceleration,
			const ct::rbd::RBDState<12>& state)
	{
		return acceleration.toStateUpdateVectorEulerXyz(state);
	}

	ct::rbd::RBDState<12> RBDStateFromVector(const StateVector& state)
	{
		ct::rbd::RBDState<12> x;
		x.fromStateVectorEulerXyz(state);
		return x;
	}

};

} // namespace HyQ

} // namespace models
} // namespace ct

#endif /* INCLUDE_CT_MODELS_HYQ_CONTACTMODEL_FORWARDZERO_SYSTEM_H_ */
