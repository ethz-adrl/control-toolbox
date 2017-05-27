/*
 * ConstraintBase.h
 * author: 			mgiftthaler@ethz.ch
 * date created: 	04.10.2016
 *
 */

#ifndef CT_OPTCON_CONSTRAINTBASE_H_
#define CT_OPTCON_CONSTRAINTBASE_H_

#include <ct/core/core.h>
#include <Eigen/Sparse>
#include <ct/core/internal/traits/TraitSelector.h>
#include <ct/core/internal/traits/TraitSelectorSpecs.h>

namespace ct {
namespace optcon {
namespace tpl {

// Rethink whether the whole class needs to be templated on SCALAR
template <size_t STATE_DIM, size_t INPUT_DIM, typename SCALAR>
class ConstraintBase {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	typedef typename ct::core::tpl::TraitSelector<SCALAR>::Trait Trait;
	typedef Eigen::Matrix<int, Eigen::Dynamic, 1> VectorXi;
	typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> VectorXs;
	typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic> MatrixXs;

	ConstraintBase(std::string name = "Unnamed") :
		tAd_(SCALAR(0)),
		xAd_(core::StateVector<STATE_DIM, SCALAR>::Zero()),
		uAd_(core::ControlVector<INPUT_DIM, SCALAR>::Zero()),
		t_(0),
		x_(core::StateVector<STATE_DIM>::Zero()),
		u_(core::ControlVector<INPUT_DIM>::Zero()),
		name_(name)
		{

		}

	ConstraintBase(const ConstraintBase& arg):
		tAd_(arg.tAd_),
		xAd_(arg.xAd_),
		uAd_(arg.uAd_),
		t_(arg.t_),
		x_(arg.x_),
		u_(arg.u_),
		lb_(arg.lb_),
		ub_(arg.ub_),
		name_(arg.name_)
		{}

	virtual ConstraintBase<STATE_DIM, INPUT_DIM, SCALAR>* clone () const = 0;

	virtual ~ConstraintBase() {}

	virtual void setTimeStateInputAd(
			const core::StateVector<STATE_DIM, SCALAR> &x,
			const core::ControlVector<INPUT_DIM, SCALAR> &u,
			const SCALAR t){
		xAd_ = x;
		uAd_ = u;
		tAd_ = t;
	}

	virtual void setTimeStateInputDouble(
			const core::StateVector<STATE_DIM> &x,
			const core::ControlVector<INPUT_DIM> &u,
			const double t
		)
	{
		x_ = x;
		u_ = u;
		t_ = t;
	}

	virtual size_t getConstraintsCount() = 0;

	// todo: make sure we have implemented size-checks wherever necessary, since working with dynamic size.
	virtual VectorXs evaluate() = 0;

	// return constraint type (either 0 for inequality or 1 for equality)
	virtual int getConstraintType() = 0;

	virtual Eigen::MatrixXd JacobianState() 
	{ 
		throw std::runtime_error("This constraint function element is not implemented for the given term."
		"Please use either auto-diff cost function or implement the analytical derivatives manually."); 
	}

	virtual Eigen::MatrixXd JacobianInput() 
	{ 
		throw std::runtime_error("This constraint function element is not implemented for the given term." 
		"Please use either auto-diff cost function or implement the analytical derivatives manually."); 
	}

	virtual Eigen::VectorXd getLowerBound() const
	{
		return lb_;
	}

	virtual Eigen::VectorXd getUpperBound() const
	{
		return ub_;
	}

	const void getName(std::string& constraintName) const { constraintName=name_; }

	void setName(const std::string constraintName) { name_=constraintName; }

	virtual size_t getNumNonZerosJacobianState()
	{
		return STATE_DIM * getConstraintsCount();
	}

	virtual size_t getNumNonZerosJacobianInput()
	{
		return INPUT_DIM * getConstraintsCount();
	}

	virtual Eigen::VectorXd jacobianStateSparse()
	{
		Eigen::VectorXd jac(Eigen::Map<Eigen::VectorXd>(JacobianState().data(), JacobianState().rows() * JacobianState().cols()));
		return jac;
	}

	virtual Eigen::VectorXd jacobianInputSparse()
	{
		Eigen::VectorXd jac(Eigen::Map<Eigen::VectorXd>(JacobianInput().data(), JacobianInput().rows() * JacobianInput().cols()));
		return jac;
	}


	virtual void sparsityPatternState(VectorXi& rows, VectorXi& cols)
	{
		genBlockIndices(getConstraintsCount(), STATE_DIM, rows, cols);

	}

	virtual void sparsityPatternInput(VectorXi& rows, VectorXi& cols)
	{
		genBlockIndices(getConstraintsCount(), INPUT_DIM, rows, cols);	
	}


protected:

	SCALAR tAd_;
	core::StateVector<STATE_DIM, SCALAR> xAd_;
	core::ControlVector<INPUT_DIM, SCALAR> uAd_;
	double t_;
	core::StateVector<STATE_DIM> x_;
	core::ControlVector<INPUT_DIM> u_;

	Eigen::VectorXd lb_; // lower bound on the constraints
	Eigen::VectorXd ub_; // upper bound on the constraints

	void genDiagonalIndices(
			const size_t num_elements,
			Eigen::VectorXi& iRow_vec,
			Eigen::VectorXi& jCol_vec)
	{
		iRow_vec.resize(num_elements);
		jCol_vec.resize(num_elements);

		size_t count = 0;

		for(size_t i = 0; i < num_elements; ++i){
			iRow_vec(count) = i;
			jCol_vec(count) = i;
			count++;
		}	
	}

	void genBlockIndices(
			const size_t num_rows,
			const size_t num_cols,
			Eigen::VectorXi& iRow_vec,
			Eigen::VectorXi& jCol_vec)
	{
		size_t num_gen_indices = num_rows*num_cols;

		iRow_vec.resize(num_gen_indices);
		jCol_vec.resize(num_gen_indices);

		size_t count = 0;

		for(size_t row = 0; row <num_rows; ++row){
			for(size_t col = 0; col < num_cols; ++col){
				iRow_vec(count) = row;
				jCol_vec(count) = col;
				count++;
			}
		}
	}


private:
	std::string name_;

};

} // namespace tpl

template<size_t STATE_DIM, size_t INPUT_DIM>
using ConstraintBase = tpl::ConstraintBase<STATE_DIM, INPUT_DIM, double>;

} // namespace optcon
} // namespace ct

#endif
