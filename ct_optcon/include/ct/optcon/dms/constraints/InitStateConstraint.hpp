#ifndef DMS_INIT_STATE_CONSTRAINT_HPP
#define DMS_INIT_STATE_CONSTRAINT_HPP

#include <ct/optcon/nlp/DiscreteConstraintBase.h>
#include <ct/optcon/dms/dms_core/OptVectorDms.hpp>

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM>
class InitStateConstraint : public DiscreteConstraintBase
{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	typedef DmsDimensions<STATE_DIM, CONTROL_DIM> DIMENSIONS;

	typedef DiscreteConstraintBase BASE;
	typedef typename DIMENSIONS::state_vector_t state_vector_t;
	
	InitStateConstraint(){}

	InitStateConstraint(
		const state_vector_t& x0,
		std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM>> w
		)
	:
		w_(w),
		x_0_(x0)
	{
		lb_.setConstant(0.0);
		ub_.setConstant(0.0);
	}

	void updateConstraint(const state_vector_t& x_i_new) {x_0_ = x_i_new;}

	virtual size_t getEvaluation(Eigen::Map<Eigen::VectorXd>& val, size_t count) override
	{
		val.segment(count, STATE_DIM) = w_->getOptimizedState(0) - x_0_;
		return count += STATE_DIM;
	}

	/* currently the constraint jacobian for the initial state constraint ist just the identity matrix*/
	virtual size_t evalConstraintJacobian(Eigen::Map<Eigen::VectorXd>& val, size_t count) override
	{
		return BASE::evalIblock(val, count, STATE_DIM);
	}

	virtual size_t getNumNonZerosJacobian() override
	{
		return (size_t) STATE_DIM;
	}

	virtual size_t genSparsityPattern(
				Eigen::Map<Eigen::VectorXi>& iRow_vec,
				Eigen::Map<Eigen::VectorXi>& jCol_vec,
				size_t indexNumber
		) override
	{
		/* The sparsity pattern for the Jacobian of the initial state constraint will be a diagonal */
		indexNumber += BASE::genDiagonalIndices(BASE::indexTotal_, w_->getStateIndex(0), STATE_DIM, iRow_vec, jCol_vec, indexNumber);
		return indexNumber;
	}

	virtual void getLowerBound(Eigen::VectorXd& c_lb) override
	{
		assert(lb_.size() > 0);
		c_lb.segment(BASE::indexTotal_, STATE_DIM) = lb_;
	}

	virtual void getUpperBound(Eigen::VectorXd& c_ub) override
	{
		assert(lb_.size() > 0);
		c_ub.segment(BASE::indexTotal_, STATE_DIM) = ub_;
	}

	virtual size_t getConstraintSize() override
	{
		return STATE_DIM;
	}

private:
	std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM>> w_;
	state_vector_t x_0_;

	//Constraint bounds
	state_vector_t lb_;	// lower bound
	state_vector_t ub_;	// upper bound
};

} // namespace optcon
} // namespace ct

#endif

