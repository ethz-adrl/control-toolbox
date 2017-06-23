/*
 * EmptyLinearConstraint.h
 * author: 			mgiftthaler@ethz.ch
 * date created: 	07.02.2017
 *
 */

#ifndef CT_OPTCON_EMPTY_LINEAR_CONSTRAINT_H_
#define CT_OPTCON_EMPTY_LINEAR_CONSTRAINT_H_

#include "LinearConstraintContainer.h"

namespace ct {
namespace optcon {


/**
 * \ingroup Constraint
 *+
 * \brief This constraint simply does nothing and returns number of constraints "0"
 *
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class EmptyLinearConstraint : public LinearConstraintContainer<STATE_DIM, INPUT_DIM>
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef EmptyLinearConstraint<STATE_DIM, INPUT_DIM>* EmptyLinearConstraint_Raw_Ptr_t;
	typedef ConstraintBase<STATE_DIM, INPUT_DIM> Base;


	/**
	 * \brief Default constructor
	 */
	EmptyLinearConstraint() {}

	/**
	 * \brief Destructor
	 *
	 * Destructor
	 */
	~EmptyLinearConstraint() {}

	/**
	 * Clones the linear constraint class
	 * @return pointer to the clone
	 */
	EmptyLinearConstraint_Raw_Ptr_t clone() const override {return new EmptyLinearConstraint();}

	/**
	 * This method retrieves the pure state constraint vector
	 * @param[out] g2  The value of the pure state constraint
	 * @param[out] nc2 The number of the active pure state constraints
	 */

	virtual void initialize() = 0;

	virtual void evaluate(Eigen::VectorXd& g2, size_t& nc2) override {nc2 = 0;}

	/**
	 * This method retrieves the Jacobian of the pure state constraints w.r.t state vector
	 * @param[out] F Jacobian of the pure state constraints w.r.t state vector
	 */
	virtual void evaluateConstraintJacobianState(Eigen::MatrixXd& F) override {};

	virtual void evaluateConstraintJacobianInput(Eigen::MatrixXd& D) override {};

	virtual void getConstraintTypes(std::vector<Eigen::VectorXd>& constraint_types) override {};

protected:
	/**
	 * This is called by setTimeStateInput() method class. It can be used for updating the class members with the new state and input.
	 */
	void update() override {};

};

} // namespace optcon
} // namespace ct

#endif /* CT_OPTCON_EmptyLinearConstraint_H_ */
