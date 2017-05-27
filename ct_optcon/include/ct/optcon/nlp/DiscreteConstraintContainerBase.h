#ifndef OPTCON_CONSTRAINT_BASE_HPP
#define OPTCON_CONSTRAINT_BASE_HPP


#include "DiscreteConstraintBase.h"

namespace ct{
namespace optcon{

class DiscreteConstraintContainerBase
{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef double Number;
	typedef Eigen::Matrix<Number, Eigen::Dynamic, 1> VectorXd;
	typedef Eigen::Matrix<int, Eigen::Dynamic, 1> VectorXi;
	typedef Eigen::Map<VectorXd> MapVecXd;
	typedef Eigen::Map<VectorXi> MapVecXi;

	DiscreteConstraintContainerBase()
	:
	constraintsCount_(0),
	nonZerosJacobianCount_(0)
	{}

	~DiscreteConstraintContainerBase(){}

	virtual void evalConstraints(Eigen::Map<Eigen::VectorXd>& c_local) = 0;

	virtual void getSparsityPattern(Eigen::Map<Eigen::VectorXi>& iRow_vec, Eigen::Map<Eigen::VectorXi>& jCol_vec, const int nnz_jac_g) = 0;

	virtual void evalSparseJacobian(Eigen::Map<Eigen::VectorXd>& val, const int nzz_jac_g) = 0;

	const size_t getConstraintsCount() const
	{
		return constraintsCount_;
	}

	const size_t getNonZerosJacobianCount() const
	{
		return nonZerosJacobianCount_;
	}

	const Eigen::VectorXd getUpperBounds() const
	{
		return c_ub_;
	}

	const Eigen::VectorXd getLowerBounds() const
	{
		return c_lb_;
	}


protected:
	size_t constraintsCount_;
	size_t nonZerosJacobianCount_;

	std::vector<std::shared_ptr<DiscreteConstraintBase>> constraints_;

	Eigen::VectorXd c_lb_;
	Eigen::VectorXd c_ub_;

};

}
}


#endif
