/*
 * Constraints.hpp
 *
 * Created: 16.12.2015
 * Author: mgiftthaler
 * *
 */


#ifndef DMS_CONSTRAINTS_HPP_
#define DMS_CONSTRAINTS_HPP_

//#define DEBUG_CONSTRAINTS

#include <Eigen/Dense>

#include <ct/core/core.h>
#include <ct/optcon/dms/dms_core/DmsDimensions.hpp>
#include <ct/optcon/dms/dms_core/OptVectorDms.hpp>
#include <ct/optcon/dms/dms_core/ShotContainer.hpp>

#include <ct/optcon/nlp/DiscreteConstraintBase.h>
#include <ct/optcon/nlp/DiscreteConstraintContainerBase.h>
#include <ct/optcon/dms/constraints/InitStateConstraint.hpp>
#include <ct/optcon/dms/constraints/ContinuityConstraint.hpp>
#include <ct/optcon/dms/constraints/TimeHorizonEqualityConstraint.hpp>
#include <ct/optcon/dms/dms_core/DmsSettings.hpp>
#include <ct/optcon/dms/constraints/ConstraintDiscretizer.hpp>


namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM>
class ConstraintsContainerDms : public DiscreteConstraintContainerBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef DmsDimensions<STATE_DIM, CONTROL_DIM> DIMENSIONS;

	typedef typename DIMENSIONS::state_vector_t state_vector_t;
	typedef typename DIMENSIONS::control_vector_t control_vector_t;

	typedef typename DIMENSIONS::state_matrix_t state_matrix_t;
	typedef typename DIMENSIONS::control_matrix_t control_matrix_t;
	typedef typename DIMENSIONS::state_control_matrix_t state_control_matrix_t;
	typedef typename DIMENSIONS::state_matrix_array_t state_matrix_array_t;
	typedef typename DIMENSIONS::state_control_matrix_array_t state_control_matrix_array_t;

	ConstraintsContainerDms(
			std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM>> w,
			std::shared_ptr<TimeGrid> timeGrid,
			std::vector<std::shared_ptr<ShotContainer<STATE_DIM, CONTROL_DIM>>> shotContainers,
			std::shared_ptr<ConstraintDiscretizer<STATE_DIM, CONTROL_DIM>> constraintsIntermediate,
			std::shared_ptr<ConstraintDiscretizer<STATE_DIM, CONTROL_DIM>> constraintsFinal,
			const state_vector_t& x0,
			const DmsSettings settings);

	~ConstraintsContainerDms() {};

	/* evaluate the constraint violation */
	virtual void evalConstraints(Eigen::Map<Eigen::VectorXd>& c_local) override;

	/* index the non-zero elements of the constraint jacobian */
	virtual void getSparsityPattern(Eigen::Map<Eigen::VectorXi>& iRow_vec, Eigen::Map<Eigen::VectorXi>& jCol_vec, const int nnz_jac_g) override;

	/* evaluate the sparse jacobian and write the results into the indexed vector "val"
	 * This function must use the same indexing functinality as the pattern generator "genIpoptSparsityIndexPattern". */
	virtual void evalSparseJacobian(Eigen::Map<Eigen::VectorXd>& val, const int nzz_jac_g) override;

	void updateTerminalConstraint(const state_vector_t& x_f_new);

	void updateInitialConstraint(const state_vector_t& x_init_new);

private:
	
	const DmsSettings settings_;

	// the constraints
	std::shared_ptr<InitStateConstraint<STATE_DIM, CONTROL_DIM>> c_init_;
	std::vector<std::shared_ptr<ShotContainer<STATE_DIM, CONTROL_DIM>>> shotContainers_;
};

#include "implementation/ConstraintsContainerDms-impl.hpp"

} // namespace optcon
} // namespace ct

#endif