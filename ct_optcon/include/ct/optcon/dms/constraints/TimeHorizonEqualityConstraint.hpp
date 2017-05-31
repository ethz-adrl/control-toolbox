#ifndef DMS_TIME_HORIZON_EQUALITY_CONSTRAINT_HPP
#define DMS_TIME_HORIZON_EQUALITY_CONSTRAINT_HPP

#include <ct/optcon/nlp/DiscreteConstraintBase.h>

/*
 * this constraint has dimension one, it is a scalar.
 *
 * form of the constraint:
 * h_1 + h_2 + ... + h_N = T_spec
 *
 * */

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM>
class TimeHorizonEqualityConstraint : public DiscreteConstraintBase
{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef DiscreteConstraintBase BASE;

	TimeHorizonEqualityConstraint(){}

	TimeHorizonEqualityConstraint(
		std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM>> w,
		std::shared_ptr<TimeGrid> timeGrid,
		DmsSettings settings
		):
		w_(w),
		timeGrid_(timeGrid),
		settings_(settings)
	{
		// lower bound is number of shot times the lower bound for each interval h
		lb_ << settings_.N_ * settings_.h_min_ - settings_.T_;
		ub_ << 0.0;

		std::cout << " ... time horizon lower bound: " << settings_.T_ + lb_ << std::endl;
		std::cout << " ... time horizon upper bound: " << settings_.T_ + ub_ << std::endl;
	}

	virtual Eigen::VectorXd eval() override
	{
		Eigen::Matrix<double, 1, 1> mat;
		mat << timeGrid_->getOptimizedTimeHorizon() - settings_.T_;
		return mat;
		// val(count) = 
		// return count += 1;
	}	

	/* currently the constraint jacobian for this constraint can be hacked as a scaled identity block of dimension 1*/
	virtual Eigen::VectorXd evalJacobian() override
	{
		Eigen::VectorXd one(settings_.N_);
		one.setConstant(1.0);
		return one;
	}

	virtual size_t getNumNonZerosJacobian() override
	{
		return settings_.N_;
	}

	virtual void genSparsityPattern(Eigen::VectorXi& iRow_vec, Eigen::VectorXi& jCol_vec) override
	{
		for(size_t i = 0; i < settings_.N_; ++i)
		{
			iRow_vec(i) = 0;
			jCol_vec(i) = w_->getTimeSegmentIndex(i);
		}
	}

	virtual Eigen::VectorXd getLowerBound() override
	{
		return lb_;
	}

	virtual Eigen::VectorXd getUpperBound() override
	{
		return ub_;
	}

	virtual size_t getConstraintSize() override
	{
		return 1;
	}

private:
	std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM>> w_;
	std::shared_ptr<TimeGrid> timeGrid_;
	DmsSettings settings_;

	//Constraint bounds
	Eigen::Matrix<double, 1, 1> lb_;
	Eigen::Matrix<double, 1, 1> ub_;
};

} // namespace optcon
} // namespace ct

#endif

