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
		lb_ = settings_.N_ * settings_.h_min_ - settings_.T_;
		ub_ = 0.0;

		std::cout << " ... time horizon lower bound: " << settings_.T_ + lb_ << std::endl;
		std::cout << " ... time horizon upper bound: " << settings_.T_ + ub_ << std::endl;
	}

	virtual size_t getEvaluation(Eigen::Map<Eigen::VectorXd>& val, size_t count) override
	{
		val(count) = timeGrid_->getOptimizedTimeHorizon() - settings_.T_;
		return count += 1;
	}	

	/* currently the constraint jacobian for this constraint can be hacked as a scaled identity block of dimension 1*/
	virtual size_t evalConstraintJacobian(Eigen::Map<Eigen::VectorXd>& val, size_t count) override
	{
		for(size_t i = 0; i < settings_.N_; i++)
		{
			val(count) = 1.0;
			count++;
		}
		return count;
	}

	virtual size_t getNumNonZerosJacobian() override
	{
		return settings_.N_;
	}

	virtual size_t genSparsityPattern(
			Eigen::Map<Eigen::VectorXi>& iRow_vec,
			Eigen::Map<Eigen::VectorXi>& jCol_vec,
			size_t indexNumber) override
	{
		for(size_t i = 0; i < settings_.N_; ++i)
		{
			iRow_vec(indexNumber) = BASE::indexTotal_;
			jCol_vec(indexNumber) = w_->getTimeSegmentIndex(i);
			indexNumber += 1;
		}
		// indexNumber += BASE::genBlockIndices(BASE::indexTotal_, w_->getTimeSegmentIndex(0), 1, settings_.N_, iRow_vec, jCol_vec, indexNumber);
		return indexNumber;
	}

	virtual void getLowerBound(Eigen::VectorXd& c_lb) override
	{
		c_lb(BASE::indexTotal_) = lb_;
	}

	virtual void getUpperBound(Eigen::VectorXd& c_ub) override
	{
		c_ub(BASE::indexTotal_) = ub_;
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
	double lb_;
	double ub_;
};

} // namespace optcon
} // namespace ct

#endif

