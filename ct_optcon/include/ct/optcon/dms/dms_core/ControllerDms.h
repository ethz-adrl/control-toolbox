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

#ifndef CT_OPTCON_DMS_DMS_CORE_CONTROLLER_H_
#define CT_OPTCON_DMS_DMS_CORE_CONTROLLER_H_

#include <Eigen/Dense>

#include <ct/core/control/Controller.h>
#include <ct/optcon/dms/dms_core/OptVectorDms.h>


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

#endif //CT_OPTCON_DMS_DMS_CORE_CONTROLLER_H_
