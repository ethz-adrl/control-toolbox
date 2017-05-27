

#ifndef CT_OPTCON_STATE_CONSTRAINT_HPP_
#define CT_OPTCON_STATE_CONSTRAINT_HPP_

#include "ConstraintBase.h"
#include <ct/core/types/StateVector.h>

namespace ct {
namespace optcon {
namespace tpl {


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
class StateConstraint : public ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	typedef typename ct::core::tpl::TraitSelector<SCALAR>::Trait Trait;
	typedef ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR> Base;
	typedef Eigen::Matrix<int, Eigen::Dynamic, 1> VectorXi;
	typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> VectorXs;
	typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic> MatrixXs;

	StateConstraint(
		const core::StateVector<STATE_DIM>& xLow,
		const core::StateVector<STATE_DIM>& xHigh)
	{
		Base::lb_.resize(STATE_DIM);
		Base::ub_.resize(STATE_DIM);
		// The terminal state constraint is treated as equality constraint, therefore, ub = lb
		Base::lb_ = xLow;
		Base::ub_ = xHigh;
	}

	virtual StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override
	{
		return new StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>(*this);
	}

	virtual size_t getConstraintsCount() override
	{
		return STATE_DIM;
	}

	virtual VectorXs evaluate() override
	{
		return this->xAd_;
	}

	virtual Eigen::MatrixXd JacobianState() override
	{
		jac_.setIdentity();
		return jac_;
	}

	virtual Eigen::MatrixXd JacobianInput() override
	{
		return Eigen::Matrix<double, STATE_DIM, CONTROL_DIM>::Zero();
	}

	virtual size_t getNumNonZerosJacobianState() override
	{
		return STATE_DIM;
	}
	virtual size_t getNumNonZerosJacobianInput() override
	{
		return 0;
	}

	virtual Eigen::VectorXd jacobianStateSparse() override
	{
		return core::StateVector<STATE_DIM>::Ones();
	}

	virtual void sparsityPatternState(VectorXi& rows, VectorXi& cols) override
	{
		this->genDiagonalIndices(STATE_DIM, rows, cols);
	}

	// return term type (either 0 for inequality or 1 for equality)
	virtual int getConstraintType() override
	{
		return 0;
	}

private:
	Eigen::Matrix<double, STATE_DIM, STATE_DIM> jac_; 
};

}

template<size_t STATE_DIM, size_t INPUT_DIM>
using StateConstraint = tpl::StateConstraint<STATE_DIM, INPUT_DIM, double>;

}
}


#endif //CT_OPTCON_CONTROL_INPUT_CONSTRAINT_HPP_