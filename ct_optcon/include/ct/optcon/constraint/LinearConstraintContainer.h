/*
 * LinearConstraintBase.h
 * author: 			mgiftthaler@ethz.ch
 * date created: 	04.10.2016
 *
 */

#ifndef CT_OPTCON_LINEARCONSTRAINTBASE_H_
#define CT_OPTCON_LINEARCONSTRAINTBASE_H_

#include "ConstraintContainerBase.h"

namespace ct {
namespace optcon {


/**
 * \ingroup Constraint
 *+
 * \brief A base function for linear constraint functions. All linear constraint functions should derive from this.
 *
 * * The LinearConstraintBase Class is the base class for defining the non-linear optimization constraints.
 *
 * @tparam STATE_DIM: Dimension of the state vector
 * @tparam INPUT_DIM: Dimension of the control input vector
 * @tparam CONSTRAINT2_DIM: Maximum number of pure state constraints
 * @tparam CONSTRAINT1_DIM: Maximum number of state-input constraints
 *
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class LinearConstraintContainer : public ConstraintContainerBase<STATE_DIM, INPUT_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef core::StateVector<STATE_DIM> state_vector_t;
	typedef core::ControlVector<INPUT_DIM> input_vector_t;

	typedef LinearConstraintContainer<STATE_DIM, INPUT_DIM>* LinearConstraintContainer_Raw_Ptr_t;
	typedef std::shared_ptr<LinearConstraintContainer<STATE_DIM, INPUT_DIM>> LinearConstraintContainer_Shared_Ptr_t;

	/**
	 * \brief Default constructor
	 */
	LinearConstraintContainer() {}


	/**
	 * \brief Copy constructor
	 */
	LinearConstraintContainer(const LinearConstraintContainer& arg)
	{}

	/**
	 * \brief Destructor
	 *
	 * Destructor
	 */
	virtual ~LinearConstraintContainer() {}

	/**
	 * Clones the linear constraint class
	 * @return pointer to the clone
	 */
	virtual LinearConstraintContainer_Raw_Ptr_t clone() const = 0;
	virtual void setTimeStateInput(const double t, const state_vector_t& x, const input_vector_t& u) = 0;

	virtual void evaluate(Eigen::VectorXd& g, size_t& count) = 0;

	virtual void getLowerBound(Eigen::VectorXd& lb, size_t& count) = 0;
	virtual void getUpperBound(Eigen::VectorXd& ub, size_t& count) = 0;

	virtual void evalJacSparse(Eigen::VectorXd& jacVec, size_t& count) = 0;
	virtual Eigen::MatrixXd evalJacDense() = 0;

	virtual void evalJacStateSparse(Eigen::VectorXd& jacVec, size_t& count) = 0;
	virtual Eigen::MatrixXd evalJacStateDense() = 0;

	virtual void evalJacInputSparse(Eigen::VectorXd& jacVec, size_t& count) = 0;
	virtual Eigen::MatrixXd evalJacInputDense() = 0;

	virtual size_t generateSparsityPatternJacobian(Eigen::VectorXi& iRows, Eigen::VectorXi& jCols) = 0;

	virtual size_t generateSparsityPatternJacobianState(Eigen::VectorXi& iRows, Eigen::VectorXi& jCols) = 0;

	virtual size_t generateSparsityPatternJacobianInput(Eigen::VectorXi& iRows, Eigen::VectorXi& jCols) = 0;

	virtual size_t getConstraintJacobianNonZeroCount() = 0;
	virtual size_t getConstraintJacobianStateNonZeroCount() = 0;
	virtual size_t getConstraintJacobianInputNonZeroCount() = 0;

	virtual size_t getConstraintCount() = 0;
	virtual void getConstraintTypes(Eigen::VectorXd& constraint_types) = 0;


protected:
	/**
	 * This is called by setTimeStateInput() method class. It can be used for updating the class members with the new state and input.
	 */
	virtual void update() = 0;
};

} // namespace optcon
} // namespace ct

#endif /* CT_OPTCON_LINEARCONSTRAINTBASE_H_ */
