

#ifndef CT_OPTCON_CONTROL_INPUT_CONSTRAINT_HPP_
#define CT_OPTCON_CONTROL_INPUT_CONSTRAINT_HPP_

#include "ConstraintBase.h"
#include <ct/core/types/ControlVector.h>
#include <ct/core/internal/traits/DoubleTrait.h>


namespace ct {
namespace optcon {
namespace tpl {


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
class ControlInputConstraint : public ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	typedef typename ct::core::tpl::TraitSelector<SCALAR>::Trait Trait;
	typedef ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR> Base;
	typedef Eigen::Matrix<int, Eigen::Dynamic, 1> VectorXi;
	typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> VectorXs;
	typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic> MatrixXs;

	ControlInputConstraint(
		const core::ControlVector<CONTROL_DIM> uLow,
		const core::ControlVector<CONTROL_DIM> uHigh)
	{
		Base::lb_.resize(CONTROL_DIM);
		Base::ub_.resize(CONTROL_DIM);
		// The terminal state constraint is treated as equality constraint, therefore, ub = lb
		Base::lb_ = uLow;
		Base::ub_ = uHigh;
	}

	virtual ControlInputConstraint<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override
	{
		return new ControlInputConstraint<STATE_DIM, CONTROL_DIM, SCALAR>(*this);
	}

	virtual size_t getConstraintsCount() override
	{
		return CONTROL_DIM;
	}

	virtual VectorXs evaluate() override
	{
		return this->uAd_;
	}

	virtual Eigen::MatrixXd JacobianState() override
	{
		return Eigen::Matrix<double, CONTROL_DIM, STATE_DIM>::Zero();
	}

	virtual Eigen::MatrixXd JacobianInput() override
	{
		jac_.setIdentity();
		return jac_;
	}

	virtual size_t getNumNonZerosJacobianState() override
	{
		return 0;
	}
	
	virtual size_t getNumNonZerosJacobianInput() override
	{
		return CONTROL_DIM;
	}

	virtual Eigen::VectorXd jacobianInputSparse() override
	{
		return core::ControlVector<CONTROL_DIM>::Ones();
	}

	virtual void sparsityPatternInput(VectorXi& rows, VectorXi& cols) override
	{
		this->genDiagonalIndices(CONTROL_DIM, rows, cols);
	}

	// return term type (either 0 for inequality or 1 for equality)
	virtual int getConstraintType() override
	{
		return 0;
	}

private:
	Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> jac_; 
};

} // namespace tpl

template<size_t STATE_DIM, size_t INPUT_DIM>
using ControlInputConstraint = tpl::ControlInputConstraint<STATE_DIM, INPUT_DIM, double>;

}
}


#endif //CT_OPTCON_CONTROL_INPUT_CONSTRAINT_HPP_