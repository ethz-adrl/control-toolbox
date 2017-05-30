/*
 * ControllerDms.h
 *
 *  Created on: 11.11.2015
 *      Author: mgiftthaler
 *
 *  This class provides an interface to the ct controllers and integrators.
 */

#ifndef DMS_CONTROLLER_H_
#define DMS_CONTROLLER_H_

#include <Eigen/Dense>

#include <ct/core/control/Controller.h>
#include <ct/optcon/dms/dms_core/OptVectorDms.hpp>


/**
 * \ingroup DMS
 *
 * \brief DMS controller class
 *
 *	Implements a controller to be handed over to a system of type "ControlledSystem". This controller applies the nominal input trajectory designed by the algorithm,
 *	which is either piecewise constant or piecewise linear between the nodes.
 *
 * @tparam STATE_DIM: Dimension of the state vector
 * @tparam INPUT_DIM: Dimension of the control input vector
 *
 */

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM>
class ControllerDms : public ct::core::Controller<STATE_DIM, CONTROL_DIM>
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef DmsDimensions<STATE_DIM, CONTROL_DIM> DIMENSIONS;
	typedef typename DIMENSIONS::state_vector_t state_vector_t;
	typedef typename DIMENSIONS::control_vector_t control_vector_t;


	ControllerDms() = delete;

	ControllerDms(
			std::shared_ptr<SplinerBase<control_vector_t>> controlSpliner,
			size_t shotIdx):
		controlSpliner_(controlSpliner),
		shotIdx_(shotIdx)
	{}

	/* Important note: when we make a copy of ControllerDms, the underlying optimization
	 *  vector instance is not copied. We are just handing over the pointer and increasing its reference count */
	ControllerDms(const ControllerDms& arg):
			controlSpliner_(arg.controlSpliner_),
			shotIdx_(arg.shotIdx_)
	{}

	virtual ~ControllerDms(){}

	virtual ControllerDms<STATE_DIM, CONTROL_DIM>* clone() const override
	{
		return new ControllerDms<STATE_DIM, CONTROL_DIM> (*this);
	}


	void computeControl(
			const state_vector_t& state,
			const double& t,
			control_vector_t&  controlAction)
	{
		controlAction = controlSpliner_->evalSpline(t, shotIdx_);
		assert(controlAction == controlAction);
	}



private:

	std::shared_ptr<SplinerBase<control_vector_t>> controlSpliner_;

	/* index of the shot to which this controller belongs */
	size_t shotIdx_;

};

} // namespace optcon
} // namespace ct

#endif
