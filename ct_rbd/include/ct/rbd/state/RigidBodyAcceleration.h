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

#ifndef _RIGIDBODYACCELERATION_H_
#define _RIGIDBODYACCELERATION_H_


namespace ct {
namespace rbd {
namespace tpl {

/**
 * @class RigidBodyAcceleration
 *
 * \ingroup State
 *
 * @brief acceleration of a rigid body
 */
template <typename SCALAR>
class RigidBodyAcceleration
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef kindr::Acceleration<SCALAR, 3> LinearAcceleration;
    typedef kindr::AngularAcceleration<SCALAR, 3> AngularAcceleration;

	RigidBodyAcceleration ()
	{
    	setZero();
	}

	RigidBodyAcceleration (LinearAcceleration& transAcceleration, AngularAcceleration& anglAcceleration):
		anglAcceleration_(anglAcceleration),
		transAcceleration_(transAcceleration)
	{}

	RigidBodyAcceleration(const Eigen::Matrix<SCALAR, 6 ,1> &in) {

		fromVector6d(in);
	}

	void fromVector6d(const Eigen::Matrix<SCALAR, 6 , 1> & in ) {
		anglAcceleration_ << in.template head<3>();
		transAcceleration_ << in.template tail<3>();
	}

	const Eigen::Matrix<SCALAR, 6 , 1> getVector6d() const {
		Eigen::Matrix<SCALAR, 6 , 1> vector6;
		vector6 << anglAcceleration_.toImplementation(), transAcceleration_.toImplementation();
		return vector6;
	}

	/// @brief get translatory acceleration
	LinearAcceleration& getTranslationalAcceleration() {return transAcceleration_;}
	/// @brief get constant translatory acceleration
	const LinearAcceleration& getTranslationalAcceleration() const {return transAcceleration_;}
	/// @brief get angular acceleration
	AngularAcceleration& getAngularAcceleration() {return anglAcceleration_;}
	/// @brief get constant angular acceleration
	const AngularAcceleration& getAngularAcceleration() const {return anglAcceleration_;}

	/// @brief get accelerations as a "Twist"
	Eigen::Matrix<SCALAR, 6, 1> accelerations() {
		Eigen::Matrix<SCALAR,6,1> out;
		out << anglAcceleration_.toImplementation(), transAcceleration_.toImplementation();
		return out;
	}

	/// @brief set translatory acceleration
	void setTranslatoryAcceleration(LinearAcceleration& transAcceleration) {
		transAcceleration_ = transAcceleration;
	}
	/// @brief set angular acceleration
	void setAngularAcceleration(AngularAcceleration& anglAcceleration) {
		anglAcceleration_ = anglAcceleration;
	}

	/// @brief set acceleration to zero
	void setZero() {
		// cannot use .setZero() from members here due to codegen
		anglAcceleration_ = AngularAcceleration(SCALAR(0.0), SCALAR(0.0), SCALAR(0.0));
		transAcceleration_= LinearAcceleration(SCALAR(0.0), SCALAR(0.0), SCALAR(0.0));
	}

	static RigidBodyAcceleration Zero() { RigidBodyAcceleration state; state.setZero(); return state; }

protected:
	AngularAcceleration anglAcceleration_;
	LinearAcceleration transAcceleration_;
};

} // namespace tpl

typedef tpl::RigidBodyAcceleration<double> RigidBodyAcceleration;

} // namespace rbd
} // namespace ct


#endif /* _RIGIDBODYACCELERATION_H_ */
