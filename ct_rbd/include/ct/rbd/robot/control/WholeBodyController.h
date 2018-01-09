/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "JointPositionPIDController.h"

namespace ct {
namespace rbd {

/**
 * @class WholeBodyController
 */
template <size_t NJOINTS>
class WholeBodyController : public ct::core::Controller<2 * 6 + 2 * NJOINTS, NJOINTS>
{
public:
    static const size_t STATE_DIM = 2 * 6 + 2 * NJOINTS;

    WholeBodyController();

    virtual ~WholeBodyController();

    virtual WholeBodyController<NJOINTS>* clone() const override;

    virtual void computeControl(const core::StateVector<STATE_DIM>& state,
        const core::Time& t,
        core::ControlVector<NJOINTS>& control) override;

    JointPositionPIDController<NJOINTS>& getJointController();


protected:
    JointPositionPIDController<NJOINTS> jointController_;
};

}  // namespace rbd
}  // namespace ct
