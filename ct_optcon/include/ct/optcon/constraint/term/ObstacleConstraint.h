

#ifndef CT_OPTCON_CONSTRAINT_OBSTACLE_HPP_
#define CT_OPTCON_CONSTRAINT_OBSTACLE_HPP_

#include "ConstraintBase.h"
#include <ct/optcon/dms/robotics_plugin/Obstacle3d.hpp>

namespace ct {
namespace optcon {
namespace tpl {


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

	virtual size_t getConstraintsCount() override
	{
		return 1;
	}

	virtual VectorXs evaluate() override
	{
		Eigen::Vector3d collision_frame_position;
		getCollisionPointPosition_(this->xAd_, collision_frame_position);

		const Eigen::Vector3d dist = collision_frame_position - obstacle_->getPosition(this->tAd_);

		Eigen::Matrix3d R = Eigen::Matrix3d::Zero(); // rotation matrix
		R = ((obstacle_->getOrientation(this->tAd_)).matrix());

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

	virtual Eigen::MatrixXd JacobianState() override
	{
		Eigen::Vector3d state;

		getCollisionPointPosition_(this->x_, state);
		const Eigen::Vector3d dist = state - obstacle_->getPosition(this->t_);

		Eigen::Matrix3d R = Eigen::Matrix3d::Zero(); // rotation matrix
		R = (obstacle_->getOrientation(this->t_)).matrix();

		Eigen::Matrix<double, 3, STATE_DIM> dFkdSi;
		getCollisionPointJacobian_(this->x_, dFkdSi);
 
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

	virtual Eigen::MatrixXd JacobianInput() override
	{
		return Eigen::Matrix<double, 1, CONTROL_DIM>::Zero();
	}

	// return term type (either 0 for inequality or 1 for equality)
	virtual int getConstraintType() override
	{
		return 0;
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