/*
 * ShotIntegratorBase.h
 *
 * Created on: 10.02.2017
 * 	   Author: mgiftthaler<mgiftthaler@ethz.ch> 
 * 
 */

#ifndef CT_DMS_SHOTINTEGRATORBASE_HPP_
#define CT_DMS_SHOTINTEGRATORBASE_HPP_


#include <ct/core/core.h>

#include <ct/optcon/dms/dms_core/DmsDimensions.h>

namespace ct{
namespace optcon{

template<size_t STATE_DIM, size_t CONTROL_DIM>
class ShotIntegratorBase {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef DmsDimensions<STATE_DIM, CONTROL_DIM> DIMENSIONS;

	typedef typename DIMENSIONS::state_vector_t  state_vector_t;
	typedef typename DIMENSIONS::control_vector_t control_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
	typedef typename DIMENSIONS::control_vector_array_t control_vector_array_t;
	typedef typename DIMENSIONS::time_array_t time_array_t;

	typedef typename DIMENSIONS::state_matrix_t state_matrix_t;
	typedef typename DIMENSIONS::control_matrix_t control_matrix_t;
	typedef typename DIMENSIONS::state_control_matrix_t state_control_matrix_t;
	typedef typename DIMENSIONS::state_matrix_array_t state_matrix_array_t;
	typedef typename DIMENSIONS::state_control_matrix_array_t state_control_matrix_array_t;

	ShotIntegratorBase(){}

	virtual ~ShotIntegratorBase(){}

	virtual void setupSystem() = 0;

	virtual void integrate(double dtInt) = 0;

	// todo fixme this is bad naming, isn't it?
	virtual void retrieveStateTrajectories(
			time_array_t& timeTraj,
			state_vector_array_t& stateTraj,
			double& cost
	) = 0;

	// todo fixme this is bad naming, isn't it?
	virtual void retrieveTrajectories(
			time_array_t& timeTraj,
			state_vector_array_t& stateTraj,
			state_matrix_array_t& dXdSiTraj,
			state_control_matrix_array_t& dXdQiTraj,
			state_control_matrix_array_t& dXdQip1Traj,
			state_vector_array_t& dXdHiTraj,
			state_vector_t& costGradientSi,
			control_vector_t& costGradientQi,
			control_vector_t& costGradientQip1,
			double& costGradientHi
	) = 0;


};

}//namespace optcon
}//namespace ct


#endif /* SHOTINTEGRATORBASE_HPP_ */
