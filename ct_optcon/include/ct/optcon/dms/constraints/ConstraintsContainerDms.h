/***********************************************************************************
Copyright (c) 2017, Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo,
Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be used
      to endorse or promote products derived from this software without specific
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/


#ifndef CT_OPTCON_DMS_CORE_CONSTRAINTS_CONSTRAINTS_CONTAINER_DMS_H_
#define CT_OPTCON_DMS_CORE_CONSTRAINTS_CONSTRAINTS_CONTAINER_DMS_H_

#include <Eigen/Dense>

#include <ct/core/core.h>
#include <ct/optcon/dms/dms_core/DmsDimensions.h>
#include <ct/optcon/dms/dms_core/OptVectorDms.h>
#include <ct/optcon/dms/dms_core/ShotContainer.h>

#include <ct/optcon/nlp/DiscreteConstraintBase.h>
#include <ct/optcon/nlp/DiscreteConstraintContainerBase.h>
#include <ct/optcon/dms/constraints/InitStateConstraint.h>
#include <ct/optcon/dms/constraints/ContinuityConstraint.h>
#include <ct/optcon/dms/constraints/TimeHorizonEqualityConstraint.h>
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
	ConstraintsContainerDms(
			std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM>> w,
			std::shared_ptr<TimeGrid> timeGrid,
			std::vector<std::shared_ptr<ShotContainer<STATE_DIM, CONTROL_DIM>>> shotContainers,
			std::shared_ptr<ConstraintDiscretizer<STATE_DIM, CONTROL_DIM>> discretizedConstraints,
			const state_vector_t& x0,
			const DmsSettings settings);

	/**
	 * @brief      Destructor
	 */
	virtual ~ConstraintsContainerDms() {};

	virtual void prepareEvaluation() override;

	virtual void prepareJacobianEvaluation() override;

	/**
	 * @brief      Updates the initial constraint
	 *
	 * @param[in]  x0    The new initial state
	 */
	void changeInitialConstraint(const state_vector_t& x0);

private:	
	const DmsSettings settings_;

	std::shared_ptr<InitStateConstraint<STATE_DIM, CONTROL_DIM>> c_init_;
	std::vector<std::shared_ptr<ShotContainer<STATE_DIM, CONTROL_DIM>>> shotContainers_;
};

#include "implementation/ConstraintsContainerDms-impl.h"

} // namespace optcon
} // namespace ct
 
#endif //CT_OPTCON_DMS_CORE_CONSTRAINTS_CONSTRAINTS_CONTAINER_DMS_H_