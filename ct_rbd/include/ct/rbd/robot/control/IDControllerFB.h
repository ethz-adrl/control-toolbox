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

#ifndef CT_RBD_INCLUDE_CT_RBD_ROBOT_CONTROL_IDCONTROLLERFB_H_
#define CT_RBD_INCLUDE_CT_RBD_ROBOT_CONTROL_IDCONTROLLERFB_H_

namespace ct {
namespace rbd {



template <class Dynamics>
class IDControllerFB
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef typename Dynamics::RBDState_t RBDState;

	IDControllerFB(
			std::shared_ptr<Dynamics> dynamics = std::shared_ptr<Dynamics>(new Dynamics()),
			const RBDState& desState = RBDState(),
			const Eigen::Matrix<double, 6, 1>& Kp = Eigen::Matrix<double, 6, 1>::Zero(),
			const Eigen::Matrix<double, 6, 1>& Kd = Eigen::Matrix<double, 6, 1>::Zero()) :
		dynamics_(dynamics),
		desState_(desState),
		Kp_(Kp),
		Kd_(Kd)
	{

	}

	void setDesiredState(const RBDState& desState)
	{
		desState_ = desState;
	}

	void setPoseGains(const Eigen::Matrix<double, 6, 1>& Kp)
	{
		Kp_ = Kp;
	}

	void setTwistGains(const Eigen::Matrix<double, 6, 1>& Kd)
	{
		Kd_ = Kd;
	}

	typename Dynamics::RBDAcceleration_t computeDesiredAcceleration(const RBDState& currBaseState)
	{
		  // compute rotation error
		  kindr::RotationQuaternionD Bcurr_q_Bdes(currBaseState.basePose().getRotationQuaternion().inverted()*desState_.basePose().getRotationQuaternion());
		  kindr::EulerAnglesXyzPD Bcurr_eul_Bdes(Bcurr_q_Bdes);

		  // desired base acceleration
		  Eigen::Matrix<double, 6, 1> aDesEigen = Eigen::Matrix<double, 6, 1>::Zero();

		  // base orientation error
		  aDesEigen.segment(0,3) = Kp_.segment(0,3).cwiseProduct(Bcurr_eul_Bdes.getUnique().toImplementation());

		  // base position error
		  Eigen::Vector3d W_positionError = (desState_.basePose().position() - currBaseState.basePose().position()).toImplementation();
		  Eigen::Vector3d W_weightedPosError =  Kp_.segment(3,3).cwiseProduct(W_positionError);
		  aDesEigen.segment(3,3) = currBaseState.basePose().rotateInertiaToBase(W_weightedPosError);

		  // base velocity error
		  aDesEigen += Kd_.cwiseProduct(desState_.base().velocities().getVector() - currBaseState.base().velocities().getVector());

		  typename Dynamics::RBDAcceleration_t aDes;
		  aDes.setZero();

		  aDes.base().fromVector6d(aDesEigen);

		  return aDes;
	}

	typename Dynamics::control_vector_t computeTorque(const RBDState& currState, const typename Dynamics::EE_in_contact_t& eeInContact, bool gravityCompensation = true)
	{
		typename Dynamics::RBDAcceleration_t aDes = computeDesiredAcceleration(currState);

		typename Dynamics::control_vector_t uId;

		if (gravityCompensation)
		{
			dynamics_->ProjectedInverseDynamics(eeInContact, currState, aDes, uId);
		} else
		{
			dynamics_->ProjectedInverseDynamicsNoGravity(eeInContact, currState, aDes, uId);
		}

		return uId;
	}


private:
	std::shared_ptr<Dynamics> dynamics_;

	RBDState desState_;

	Eigen::Matrix<double, 6, 1> Kp_;
	Eigen::Matrix<double, 6, 1> Kd_;
};


}
}


#endif /* CT_RBD_INCLUDE_CT_RBD_ROBOT_CONTROL_IDCONTROLLERFB_H_ */
