#ifndef STABILISER_BASE_HPP_
#define STABILISER_BASE_HPP_

#include <math.h>

#include <ct/optcon/dms/stabilisation/StabDimensions.hpp>
#include <ct/optcon/lqr/FHDTLQR.hpp>

#include <ct/core/types/StateVector.h>
#include <ct/core/systems/ControlledSystem.h>
#include <ct/core/systems/linear/LinearSystem.h>
#include <ct/core/systems/linear/SystemLinearizer.h>
#include <ct/core/types/trajectories/FeedbackArray.h>

#include <ct/core/control/StateFeedbackController.h>
#include <ct/core/control/ConstantController.h>

#include <ct/core/types/trajectories/spliner/TrajectoryZohSpliner.hpp>
#include <ct/core/types/trajectories/spliner/TrajectoryLinearSpliner.hpp>

#include <ct/optcon/costfunction/CostFunctionQuadraticTracking.hpp>
#include <ct/optcon/costfunction/CostFunctionQuadratic.hpp>
#include <ct/optcon/dms/stabilisation/StabilisationCommon.hpp>


namespace ct {
namespace optcon {


template <size_t STATE_DIM, size_t CONTROL_DIM>
class StabiliserBase
{

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef StabDimensions<STATE_DIM, CONTROL_DIM> DIMENSIONS;

	typedef typename DIMENSIONS::state_vector_t state_vector_t;
	typedef typename DIMENSIONS::control_vector_t control_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
	typedef typename DIMENSIONS::control_vector_array_t control_vector_array_t;
	typedef typename DIMENSIONS::time_array_t time_array_t;

	typedef typename DIMENSIONS::state_matrix_t state_matrix_t;
	typedef typename DIMENSIONS::control_matrix_t control_matrix_t;
	typedef typename DIMENSIONS::state_control_matrix_t state_control_matrix_t;
	typedef typename DIMENSIONS::state_matrix_array_t state_matrix_array_t;
	typedef typename DIMENSIONS::state_control_matrix_array_t state_control_matrix_array_t;
	typedef typename DIMENSIONS::control_feedback_t control_feedback_t;
	typedef typename DIMENSIONS::control_feedback_array_t control_feedback_array_t;


	StabiliserBase(
			std::shared_ptr<optcon::CostFunctionQuadraticTracking<STATE_DIM, CONTROL_DIM> > costFunction,
			stab_settings settings,
			std::shared_ptr<core::ControlledSystem<STATE_DIM, CONTROL_DIM>> controlledSystem,
			std::shared_ptr<core::LinearSystem<STATE_DIM, CONTROL_DIM>> linearSystem = nullptr
	) :
	settings_(settings),
	costFunction_(costFunction)
	{
		controlledSystem_ = std::shared_ptr<core::ControlledSystem<STATE_DIM, CONTROL_DIM>> (controlledSystem->clone());

		if(linearSystem)
			linearSystem_ = linearSystem;
		else
			linearSystem_ = std::shared_ptr<core::SystemLinearizer<STATE_DIM, CONTROL_DIM>> (
					new core::SystemLinearizer<STATE_DIM, CONTROL_DIM> (controlledSystem_));

		integrator_  = std::shared_ptr<core::IntegratorRK4<STATE_DIM>> (
				new core::IntegratorRK4<STATE_DIM> (controlledSystem_));

		switch (settings.stateSpliner_)
		{
			case ZERO_ORDER_HOLD:
			{
				stateSpliner_ = std::shared_ptr<core::TrajectoryZohSpliner<state_vector_t>> (new core::TrajectoryZohSpliner<state_vector_t> ());
				break;
			}
			case PIECEWISE_LINEAR:
			{
				stateSpliner_ = std::shared_ptr<core::TrajectoryLinearSpliner<state_vector_t>> (new core::TrajectoryLinearSpliner<state_vector_t> ());
				break;
			}
			default:
				throw std::runtime_error("... unknown state interpolation type in stabilization request.");
		}

		switch (settings.inputSpliner_)
		{
			case ZERO_ORDER_HOLD:
			{
				controlSpliner_ = std::shared_ptr<core::TrajectoryZohSpliner<control_vector_t>> (new core::TrajectoryZohSpliner<control_vector_t> ());
				break;
			}
			case PIECEWISE_LINEAR:
			{
				controlSpliner_ = std::shared_ptr<core::TrajectoryLinearSpliner<control_vector_t>> (new core::TrajectoryLinearSpliner<control_vector_t> ());
				break;
			}
			default:
				throw std::runtime_error("... unknown control interpolation type in stabilization request.");
		}
	}

	virtual ~StabiliserBase(){}

	virtual void updateNominalCostVectors(
		const state_vector_t& x_nom,
		const state_vector_t& x_des
		) = 0;

	virtual bool stabilise(
			const state_vector_array_t& x_traj,
			const control_vector_array_t& u_traj,
			const time_array_t& t_traj,
			const state_matrix_t& Q_stab,
			const control_matrix_t& R_stab,
			const state_matrix_t& Q_final_stab,
			const double dt_sim,
			const state_vector_t& current_x
	) = 0;

	virtual const state_vector_array_t retrieveStateRollout() const = 0;

	virtual const control_vector_array_t retrieveControlRollout() const = 0;

	virtual const time_array_t retrieveTimeRollout() const = 0;

	virtual const control_feedback_array_t retrieveFeedbackGains() const = 0;

	virtual const control_vector_array_t retrieveFeedforwardGains() const = 0;


protected:

	std::shared_ptr<core::ControlledSystem<STATE_DIM, CONTROL_DIM>> controlledSystem_;
	std::shared_ptr<core::LinearSystem<STATE_DIM, CONTROL_DIM>> linearSystem_;

	std::shared_ptr<core::TrajectorySplinerBase<state_vector_t>> stateSpliner_;
	std::shared_ptr<core::TrajectorySplinerBase<control_vector_t>> controlSpliner_;

	std::shared_ptr<core::IntegratorBase<STATE_DIM>> integrator_;
	stab_settings settings_;
	// std::shared_ptr<optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM> > costFunction_;
	std::shared_ptr<optcon::CostFunctionQuadraticTracking<STATE_DIM, CONTROL_DIM> > costFunction_;

	control_feedback_array_t K_traj_;
	control_vector_array_t u_ff_;
};

} // namespace optcon
} // namespace ct

#endif
