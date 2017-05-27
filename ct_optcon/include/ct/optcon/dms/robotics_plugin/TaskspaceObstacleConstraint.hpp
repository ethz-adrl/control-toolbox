#ifndef CONSTRAINTS_TASKSPACE_OBSTACLE_CONSTRAINT_HPP
#define CONSTRAINTS_TASKSPACE_OBSTACLE_CONSTRAINT_HPP

#include <ct/optcon/dms/robotics_plugin/Obstacle3d.hpp>
#include <ct/optcon/dms/constraints/ConstraintBase.hpp>

namespace ct {
namespace optcon {

template<size_t STATE_DIM, size_t CONTROL_DIM>
class TaskspaceObstacleConstraint: public ConstraintBase<STATE_DIM, CONTROL_DIM>
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef ConstraintBase<STATE_DIM, CONTROL_DIM> BASE;

	typedef DmsDimensions<STATE_DIM, CONTROL_DIM> DIMENSIONS;
	typedef typename DIMENSIONS::state_vector_t state_vector_t;
	typedef typename DIMENSIONS::control_vector_t control_vector_t;

	TaskspaceObstacleConstraint(){}

	TaskspaceObstacleConstraint(
					size_t shotIndex,
					std::shared_ptr<Obstacle3d> obstacle,
					std::function<void (const state_vector_t&, Eigen::Vector3d&)> getPosition,
					std::function<void (const state_vector_t&, Eigen::Matrix<double, 3, STATE_DIM>&)> getJacobian
		):
		BASE(shotIndex),
		obstacle_(obstacle),
		getCollisionPointPosition_(getPosition),
		getCollisionPointJacobian_(getJacobian)
		{
			lb_ = 0.0;
			ub_ = std::numeric_limits<double>::max();

			switch(obstacle_->type())
			{
				case Ellipsoidal3d:
				{
					selectionMatrix_squared_.setZero();

					for(size_t i = 0; i< dim_; i++)
					{
						if(obstacle_->halfAxes()(i) > 1e-8)
							selectionMatrix_squared_(i,i) = 1.0 / (obstacle_->halfAxes()(i)*obstacle_->halfAxes()(i));
					}

					break;
				}
				case SimpleCylinderOrSphere3d:
				{
					/*
					 * nothing is happening here - since selectionMatrix_squared_ is only required for ellipsoidal obstacles
					 * */
					break;
				}
			}
		}

	virtual size_t getEvaluation(Eigen::Map<Eigen::VectorXd>& val, size_t count) override
	{
		Eigen::Vector3d collision_frame_position;
		getCollisionPointPosition_(BASE::w_->getState(BASE::shotIndex()), collision_frame_position);
		double time = BASE::w_->getTimeGridPtr()->getShotStartTime(BASE::shotIndex());

		const Eigen::Vector3d dist = collision_frame_position - obstacle_->getPosition(time);

		Eigen::Matrix3d R; // rotation matrix
		R.setZero();
		R = ((obstacle_->getOrientation(time)).matrix());

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

		val(count) = valLocal;
		return count += 1;
	}

	// TODO: state the equations here for both different cases
	virtual size_t evalConstraintJacobian(Eigen::Map<Eigen::VectorXd>& val, size_t count) override
	{
		Eigen::Vector3d state;

		getCollisionPointPosition_(BASE::w_->getState(BASE::shotIndex()), state);
		double time = BASE::w_->getTimeGridPtr()->getShotStartTime(BASE::shotIndex());
		const Eigen::Vector3d dist = state - obstacle_->getPosition(time);

		Eigen::Matrix3d R; // rotation matrix
		R.setZero();
		R = (obstacle_->getOrientation(time)).matrix();

		state_vector_t jac_diag;
		Eigen::Matrix<double, 3, STATE_DIM> dFkdSi;
		getCollisionPointJacobian_(BASE::w_->getState(BASE::shotIndex()), dFkdSi);
 
		switch(obstacle_->type())
		{
		case Ellipsoidal3d:
		{
			jac_diag = 2 * dist.transpose() * R * selectionMatrix_squared_ * R.transpose() * dFkdSi;
			break;
		}
		case SimpleCylinderOrSphere3d:
		{
			jac_diag = 2 * dist.transpose() * R * obstacle_->selectionMatrix() * R.transpose() * dFkdSi;
			break;
		}
		}

		// fill in
		for(size_t i = 0; i < STATE_DIM; i++)
		{
			val(count) = jac_diag(i);
			count++;
		}

		return (size_t) count;
	}

	/* the obstacle constraint is scalar and always a function of the state (e.g. generalized coordinates q). Hence,
	 * the Jacobian has at most STATE_DIM non-zero elements. */
	virtual size_t getNumNonZerosJacobian() override
	{
		return (size_t) STATE_DIM;
	}

	virtual size_t genSparsityPattern(
		Eigen::Map<Eigen::VectorXi>& iRow_vec,
		Eigen::Map<Eigen::VectorXi>& jCol_vec,
		size_t indexNumber) override
	{
		indexNumber += BASE::genBlockIndices(BASE::c_index(),  BASE::w_->getStateIndex(BASE::shotIndex()), 1, STATE_DIM, iRow_vec, jCol_vec, indexNumber);
		return indexNumber;
	}

	virtual void getLowerBound(Eigen::VectorXd& c_lb) override
	{
		c_lb(BASE::c_index()) = lb_;
	}

	virtual void getUpperBound(Eigen::VectorXd& c_ub) override
	{
		c_ub(BASE::c_index()) = ub_;
	}

	virtual size_t getConstraintSize() override
	{
		return 1;
	}


private:
	//Constraint bounds
	double lb_;
	double ub_;

	std::shared_ptr<Obstacle3d> obstacle_;

	Eigen::Matrix3d selectionMatrix_squared_;

	const size_t dim_ = 3;

	std::function<void (const state_vector_t&, Eigen::Vector3d&)> getCollisionPointPosition_;
	std::function<void (const state_vector_t&, Eigen::Matrix<double, 3, STATE_DIM>&)> getCollisionPointJacobian_;

};

} // namespace optcon
} // namespace ct


#endif
