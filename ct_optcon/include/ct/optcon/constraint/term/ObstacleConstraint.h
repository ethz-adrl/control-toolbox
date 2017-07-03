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

#ifndef CT_OPTCON_CONSTRAINT_TERM_CONSTRAINT_OBSTACLE_HPP_
#define CT_OPTCON_CONSTRAINT_TERM_CONSTRAINT_OBSTACLE_HPP_

#include "ConstraintBase.h"
#include <ct/optcon/dms/robotics_plugin/Obstacle3d.hpp>

namespace ct {
namespace optcon {
namespace tpl {


/**
 * @brief      Class for obstacle constraint.
 *
 * @tparam     STATE_DIM  The state dimension
 * @tparam     INPUT_DIM  The control dimension
 * @tparam     SCALAR     The Scalar type
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
class ObstacleConstraint : public ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	typedef typename ct::core::tpl::TraitSelector<SCALAR>::Trait Trait;
	typedef ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR> Base;
	typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> VectorXs;
	typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic> MatrixXs;


	// This constructor will be removed later, just for debugging
	ObstacleConstraint()
	{
		val_.resize(1);
		jac_.resize(1, STATE_DIM);
		Base::lb_.resize(1);
		Base::ub_.resize(1);
		Base::lb_(0) = 0.0;
		Base::ub_(0) = std::numeric_limits<double>::max();
	}

	ObstacleConstraint(
			std::shared_ptr<Obstacle3d> obstacle,
			std::function<void (const core::StateVector<STATE_DIM>&, Eigen::Vector3d&)> getPosition,
			std::function<void (const core::StateVector<STATE_DIM>&, Eigen::Matrix<double, 3, STATE_DIM>&)> getJacobian)
	:
		obstacle_(obstacle),
		getCollisionPointPosition_(getPosition),
		getCollisionPointJacobian_(getJacobian)
	{
		this->lb_.resize(1);
		this->ub_.resize(1);
		this->lb_(0) = 0.0;
		this->ub_(0) = std::numeric_limits<double>::max();

		selectionMatrix_squared_.setZero();
		if(obstacle_->type() == Ellipsoidal3d)
		{
				for(size_t i = 0; i< 3; i++)
				{
					if(obstacle_->halfAxes()(i) > 1e-8)
						selectionMatrix_squared_(i,i) = 1.0 / (obstacle_->halfAxes()(i)*obstacle_->halfAxes()(i));
				}			
		}
	}	

	virtual ObstacleConstraint<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override
	{
		return new ObstacleConstraint<STATE_DIM, CONTROL_DIM, SCALAR>(*this);
	}

	ObstacleConstraint(const ObstacleConstraint& arg):
		Base(arg),
		obstacle_(arg.obstacle_),
		selectionMatrix_squared_(arg.selectionMatrix_squared_),
		getCollisionPointPosition_(arg.getCollisionPointPosition_),
		getCollisionPointJacobian_(arg.getCollisionPointJacobian_)
		{}

	virtual size_t getConstraintSize() const override
	{
		return 1;
	}

	virtual Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> evaluate(const Eigen::Matrix<SCALAR, STATE_DIM, 1> &x, const Eigen::Matrix<SCALAR, CONTROL_DIM, 1> &u, const SCALAR t) override
	{
		Eigen::Vector3d collision_frame_position;
		getCollisionPointPosition_(x, collision_frame_position);

		const Eigen::Vector3d dist = collision_frame_position - obstacle_->getPosition(t);

		Eigen::Matrix3d R = Eigen::Matrix3d::Zero(); // rotation matrix
		R = ((obstacle_->getOrientation(x)).matrix());

		double valLocal = 0.0;

		switch(obstacle_->type())
		{
			case Ellipsoidal3d:
			{
				valLocal = dist.transpose() * ( R* selectionMatrix_squared_* R.transpose()) * dist - 1.0;
				break;
			}
			case SimpleCylinderOrSphere3d:
			{
				valLocal = dist.transpose() * R * obstacle_->selectionMatrix() * R.transpose() * dist
						- obstacle_->radius()*obstacle_->radius();
				break;
			}
		}
		val_(0) = valLocal;
		return val_;	
	}

	virtual Eigen::MatrixXd jacobianState(const Eigen::Matrix<double, STATE_DIM, 1> &x, const Eigen::Matrix<double, CONTROL_DIM, 1> &u, const double t) override
	{
		Eigen::Vector3d state;

		getCollisionPointPosition_(x, state);
		const Eigen::Vector3d dist = state - obstacle_->getPosition(t);

		Eigen::Matrix3d R = Eigen::Matrix3d::Zero(); // rotation matrix
		R = (obstacle_->getOrientation(t)).matrix();

		Eigen::Matrix<double, 3, STATE_DIM> dFkdSi;
		getCollisionPointJacobian_(x, dFkdSi);
 
		switch(obstacle_->type())
		{
			case Ellipsoidal3d:
			{
				jac_ = 2 * dist.transpose() * R * selectionMatrix_squared_ * R.transpose() * dFkdSi;
				break;
			}
			case SimpleCylinderOrSphere3d:
			{
				jac_ = 2 * dist.transpose() * R * obstacle_->selectionMatrix() * R.transpose() * dFkdSi;
				break;
			}
		}

		return jac_;
	}

	virtual Eigen::MatrixXd jacobianInput(const Eigen::Matrix<double, STATE_DIM, 1> &x, const Eigen::Matrix<double, CONTROL_DIM, 1> &u, const double t) override
	{
		return Eigen::Matrix<double, 1, CONTROL_DIM>::Zero();
	}


private:
	std::shared_ptr<Obstacle3d> obstacle_;

	Eigen::Matrix3d selectionMatrix_squared_;

	std::function<void (const core::StateVector<STATE_DIM>&, Eigen::Vector3d&)> getCollisionPointPosition_;
	std::function<void (const core::StateVector<STATE_DIM>&, Eigen::Matrix<double, 3, STATE_DIM>&)> getCollisionPointJacobian_;

	core::StateVector<1, SCALAR> val_;
	Eigen::Matrix<SCALAR, 1, STATE_DIM> jac_;
};

}

template<size_t STATE_DIM, size_t INPUT_DIM>
using ObstacleConstraint = tpl::ObstacleConstraint<STATE_DIM, INPUT_DIM, double>;

}
}


#endif //CT_OPTCON_CONSTRAINT_OBSTACLE_HPP_