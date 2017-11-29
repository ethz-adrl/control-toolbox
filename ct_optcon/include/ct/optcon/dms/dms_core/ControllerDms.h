/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <Eigen/Dense>

#include <ct/optcon/dms/dms_core/OptVectorDms.h>


namespace ct {
namespace optcon {


/*!
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
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class ControllerDms : public ct::core::Controller<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef DmsDimensions<STATE_DIM, CONTROL_DIM, SCALAR> DIMENSIONS;
    typedef typename DIMENSIONS::state_vector_t state_vector_t;
    typedef typename DIMENSIONS::control_vector_t control_vector_t;


    ControllerDms() = delete;

    ControllerDms(std::shared_ptr<SplinerBase<control_vector_t, SCALAR>> controlSpliner, size_t shotIdx)
        : controlSpliner_(controlSpliner), shotIdx_(shotIdx)
    {
    }


    ControllerDms(const ControllerDms& arg) : controlSpliner_(arg.controlSpliner_), shotIdx_(arg.shotIdx_) {}
    virtual ~ControllerDms() {}
    virtual ControllerDms<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override
    {
        return new ControllerDms<STATE_DIM, CONTROL_DIM, SCALAR>(*this);
    }


    void computeControl(const state_vector_t& state, const SCALAR& t, control_vector_t& controlAction)
    {
        controlAction = controlSpliner_->evalSpline(t, shotIdx_);
        assert(controlAction == controlAction);
    }

    virtual core::ControlMatrix<CONTROL_DIM, SCALAR> getDerivativeU0(const state_vector_t& state,
        const SCALAR time) override
    {
        return controlSpliner_->splineDerivative_q_i(time, shotIdx_);
    }

    virtual core::ControlMatrix<CONTROL_DIM, SCALAR> getDerivativeUf(const state_vector_t& state,
        const SCALAR time) override
    {
        return controlSpliner_->splineDerivative_q_iplus1(time, shotIdx_);
    }


private:
    std::shared_ptr<SplinerBase<control_vector_t, SCALAR>> controlSpliner_;

    /* index of the shot to which this controller belongs */
    size_t shotIdx_;
};

}  // namespace optcon
}  // namespace ct
