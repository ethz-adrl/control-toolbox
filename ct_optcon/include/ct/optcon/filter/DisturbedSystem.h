/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t DIST_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class AugmentedController : public ct::core::Controller<STATE_DIM + DIST_DIM, CONTROL_DIM, SCALAR>
{
public:
    AugmentedController(std::shared_ptr<ct::core::Controller<STATE_DIM, CONTROL_DIM, SCALAR>> controller = nullptr)
        : controller_(controller)
    {
    }

    AugmentedController* clone() const override {
        throw std::runtime_error("Not implemented yet!");
    }

    void computeControl(const ct::core::StateVector<STATE_DIM + DIST_DIM, SCALAR>& state,
        const SCALAR& t,
        ct::core::ControlVector<CONTROL_DIM, SCALAR>& controlAction) override
    {
        if (!controller_) throw std::runtime_error("Controller not set!");
        controller_->computeControl(state.head(STATE_DIM), t, controlAction);
    }

    ct::core::ControlMatrix<CONTROL_DIM, SCALAR> getDerivativeU0(
        const ct::core::StateVector<STATE_DIM + DIST_DIM, SCALAR>& state,
        const SCALAR time) override
    {
        if (!controller_) throw std::runtime_error("Controller not set!");
        return controller_->getDerivativeU0(state.head(STATE_DIM), time);
    }
    ct::core::ControlMatrix<CONTROL_DIM, SCALAR> getDerivativeUf(
        const ct::core::StateVector<STATE_DIM + DIST_DIM, SCALAR>& state,
        const SCALAR time) override
    {
        if (!controller_) throw std::runtime_error("Controller not set!");
        return controller_->getDerivativeUf(state.head(STATE_DIM), time);
    }

    void setController(std::shared_ptr<ct::core::Controller<STATE_DIM, CONTROL_DIM, SCALAR>> controller)
    {
        controller_ = controller;
    }

private:
    std::shared_ptr<ct::core::Controller<STATE_DIM, CONTROL_DIM, SCALAR>> controller_;
};

template <size_t STATE_DIM, size_t DIST_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class DisturbedSystem : public ct::core::ControlledSystem<STATE_DIM + DIST_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    DisturbedSystem()
        : ct::core::ControlledSystem<STATE_DIM + DIST_DIM, CONTROL_DIM, SCALAR>(ct::core::SYSTEM_TYPE::GENERAL)
    {
    }

    DisturbedSystem(std::shared_ptr<ct::core::Controller<STATE_DIM, CONTROL_DIM, SCALAR>> controller)
        : ct::core::ControlledSystem<STATE_DIM + DIST_DIM, CONTROL_DIM, SCALAR>(
              std::shared_ptr<AugmentedController<STATE_DIM, DIST_DIM, CONTROL_DIM, SCALAR>>(
                  new AugmentedController<STATE_DIM, DIST_DIM, CONTROL_DIM, SCALAR>(controller)),
              ct::core::SYSTEM_TYPE::GENERAL)
    {
        this->controlAction_.setZero();
    }

    // Not overriden. This is a completely new method.
    void setController(const std::shared_ptr<ct::core::Controller<STATE_DIM, CONTROL_DIM, SCALAR>>& controller)
    {
        std::dynamic_pointer_cast<AugmentedController<STATE_DIM, DIST_DIM, CONTROL_DIM, SCALAR>>(this->controller_)
            ->setController(controller);
    }

    virtual void computeControlledDynamics(const ct::core::StateVector<STATE_DIM + DIST_DIM, SCALAR>& state,
        const SCALAR& t,
        const ct::core::ControlVector<CONTROL_DIM, SCALAR>& control,
        ct::core::StateVector<STATE_DIM + DIST_DIM, SCALAR>& derivative) = 0;
};

}  // optcon
}  // ct
