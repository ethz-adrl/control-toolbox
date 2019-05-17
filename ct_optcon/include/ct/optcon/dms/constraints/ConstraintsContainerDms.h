/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/


#pragma once

#include <Eigen/Dense>

#include <ct/optcon/dms/dms_core/DmsDimensions.h>
#include <ct/optcon/dms/dms_core/OptVectorDms.h>
#include <ct/optcon/dms/dms_core/ShotContainer.h>

#include <ct/optcon/nlp/DiscreteConstraintBase.h>
#include <ct/optcon/nlp/DiscreteConstraintContainerBase.h>
#include <ct/optcon/dms/constraints/InitStateConstraint.h>
#include <ct/optcon/dms/constraints/ContinuityConstraint.h>
#include <ct/optcon/dms/dms_core/DmsSettings.h>
#include <ct/optcon/dms/constraints/ConstraintDiscretizer.h>


namespace ct {
namespace optcon {

/**
 * @ingroup    DMS
 *
 * @brief      Container class for the constraints used in DMS
 *
 * @tparam     STATE_DIM    The state dimension
 * @tparam     CONTROL_DIM  The control dimension
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class ConstraintsContainerDms : public tpl::DiscreteConstraintContainerBase<SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef DmsDimensions<STATE_DIM, CONTROL_DIM, SCALAR> DIMENSIONS;

    typedef typename DIMENSIONS::state_vector_t state_vector_t;
    typedef typename DIMENSIONS::control_vector_t control_vector_t;

    typedef typename DIMENSIONS::state_matrix_t state_matrix_t;
    typedef typename DIMENSIONS::control_matrix_t control_matrix_t;
    typedef typename DIMENSIONS::state_control_matrix_t state_control_matrix_t;
    typedef typename DIMENSIONS::state_matrix_array_t state_matrix_array_t;
    typedef typename DIMENSIONS::state_control_matrix_array_t state_control_matrix_array_t;

    /**
	 * @brief      Custom constructor
	 *
	 * @param[in]  w                        The optimization variables
	 * @param[in]  timeGrid                 The time grid
	 * @param[in]  shotContainers           The shot containers
	 * @param[in]  constraintsIntermediate  The intermediate constraints
	 * @param[in]  constraintsFinal         The final constraints
	 * @param[in]  x0                       The initial state
	 * @param[in]  settings                 The dms settings
	 */
    ConstraintsContainerDms(std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>> w,
        std::shared_ptr<tpl::TimeGrid<SCALAR>> timeGrid,
        std::vector<std::shared_ptr<ShotContainer<STATE_DIM, CONTROL_DIM, SCALAR>>> shotContainers,
        std::shared_ptr<ConstraintDiscretizer<STATE_DIM, CONTROL_DIM, SCALAR>> discretizedConstraints,
        const state_vector_t& x0,
        const DmsSettings settings);

    /**
	 * @brief      Destructor
	 */
    ~ConstraintsContainerDms() override = default;

    void prepareEvaluation() override;

    void prepareJacobianEvaluation() override;

    /**
	 * @brief      Updates the initial constraint
	 *
	 * @param[in]  x0    The new initial state
	 */
    void changeInitialConstraint(const state_vector_t& x0);

private:
    const DmsSettings settings_;

    std::shared_ptr<InitStateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>> c_init_;
    std::vector<std::shared_ptr<ShotContainer<STATE_DIM, CONTROL_DIM, SCALAR>>> shotContainers_;
};

#include "implementation/ConstraintsContainerDms-impl.h"

}  // namespace optcon
}  // namespace ct
