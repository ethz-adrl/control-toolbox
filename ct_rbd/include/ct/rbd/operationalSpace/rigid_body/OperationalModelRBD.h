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

#ifndef CT_RIGIDBODYMODEL_H_
#define CT_RIGIDBODYMODEL_H_

#include <ct/rbd/robot/jacobian/FrameJacobian.h>
#include <ct/rbd/robot/kinematics/EndEffector.h>
#include "OperationalModelBase.h"

namespace ct {
namespace rbd {
namespace tpl {

/**
 * \ingroup OS
 * \brief This is a class for expressing the RBD equations of an articulated robot as an operational model.
 * It uses the RBDContainer class as an interface to access the generated code for an articulated robot.
 */
template <class RBDContainer, size_t NUM_CONTACTPOINTS, typename SCALAR = double>
class OperationalModelRBD : public OperationalModelBase<RBDContainer::NJOINTS+6,RBDContainer::NJOINTS,NUM_CONTACTPOINTS>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef Eigen::Matrix<SCALAR, 3, 3> Matrix3s;
	typedef Eigen::Matrix<SCALAR, 3, 1> Vector3s;
	
	enum {
		NUM_OUTPUTS = RBDContainer::NJOINTS+6,
		NUM_JOINTS  = RBDContainer::NJOINTS,
	};

	typedef std::shared_ptr<OperationalModelRBD<RBDContainer,NUM_CONTACTPOINTS> > ptr;
	typedef OperationalModelBase<NUM_OUTPUTS,NUM_JOINTS,NUM_CONTACTPOINTS> Base;
	typedef typename Base::state_t state_t;

	OperationalModelRBD(const typename RBDContainer::Ptr_t& rbdContainerPtr, const std::array<EndEffector<NUM_JOINTS>,NUM_CONTACTPOINTS>& endEffectorArray)
	: rbdContainerPtr_(rbdContainerPtr),
	  endEffectorArray_(endEffectorArray)
	{}

	~OperationalModelRBD() {}

	/*!
	 * This method updates the class member variables using the current state.
	 * @param state The current state
	 */
	void update(const state_t& state) override {

		Base::state_ = state;

		// rotation matrix
		Matrix3s o_R_b = state.basePose().getRotationMatrix().toImplementation();

		// Mass matrix
		Base::M_ = rbdContainerPtr_->jSim().update(state.jointPositions());
		Base::MInverse_ = Base::M_.ldlt().solve(Eigen::MatrixXd::Identity(18,18));

		rbdContainerPtr_->inverseDynamics().setJointStatus(state.jointPositions());
		Eigen::Matrix<SCALAR,6,1>  baseWrench;
		Eigen::Matrix<SCALAR,12,1> jointForces;

		// centrifugal vector
		Eigen::Matrix<SCALAR,6,1> trunkVel;
		trunkVel << state.baseLocalAngularVelocity().toImplementation(), state.baseLinearVelocity().toImplementation();
		rbdContainerPtr_->inverseDynamics().C_terms_fully_actuated(baseWrench, jointForces, trunkVel, state.jointVelocities());
		Base::C_ << baseWrench, jointForces;

		// gravity vector
		rbdContainerPtr_->inverseDynamics().G_terms_fully_actuated(baseWrench, jointForces, state.basePose().computeGravityB6D());
		Base::G_ << baseWrench, jointForces;

		// Selection matrix
		Base::S_ << Eigen::MatrixXd::Zero(12,6), Eigen::MatrixXd::Identity(12,12);

		// contact jacobians
		for (size_t j=0; j<NUM_CONTACTPOINTS; j++)  {

			Vector3s b_r_f = rbdContainerPtr_->getEEPositionInBase(j, state.jointPositions()).toImplementation();
			Eigen::Matrix<SCALAR,3,NUM_JOINTS> b_J_f = rbdContainerPtr_->getJacobianBaseEEbyId(j, state.jointPositions()).template bottomRows<3>();

			FrameJacobian<NUM_JOINTS, SCALAR>::FromBaseJacToInertiaJacTranslation(o_R_b, b_r_f, b_J_f, Base::AllJc_[j]);
		} // end of j loop

	}

private:


	typename RBDContainer::Ptr_t rbdContainerPtr_;
	std::array<EndEffector<NUM_JOINTS>,NUM_CONTACTPOINTS> endEffectorArray_;
};

} // namespace tpl

template <class RBDContainer, size_t NUM_CONTACTPOINTS>
using OperationalModelRBD = tpl::OperationalModelRBD<RBDContainer, NUM_CONTACTPOINTS, double>;

}  // end of rbd namespace
}  // end of os namespace

#endif /* CT_RIGIDBODYMODEL_H_ */
