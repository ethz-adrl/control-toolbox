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

	virtual Eigen::VectorXd eval() override
	{
		return w_->getOptimizedState(0) - x_0_;
	}

	virtual Eigen::VectorXd evalSparseJacobian() override
	{
		return state_vector_t::Ones();
	}

	virtual size_t getNumNonZerosJacobian() override
	{
		return (size_t) STATE_DIM;
	}

	virtual void genSparsityPattern(Eigen::VectorXi& iRow_vec, Eigen::VectorXi& jCol_vec) override
	{
		size_t indexNumber = 0;
		indexNumber += BASE::genDiagonalIndices(w_->getStateIndex(0), STATE_DIM, iRow_vec, jCol_vec, indexNumber);
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

