/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t DIST_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class DisturbedSystemController : public ct::core::Controller<STATE_DIM + DIST_DIM, CONTROL_DIM, SCALAR>
{
public:
    static const size_t AUGMENTED_DIM = STATE_DIM + DIST_DIM;

    DisturbedSystemController(
        std::shared_ptr<ct::core::Controller<STATE_DIM, CONTROL_DIM, SCALAR>> controller = nullptr);

    DisturbedSystemController(const DisturbedSystemController& other);
    DisturbedSystemController* clone() const override;
    void computeControl(const ct::core::StateVector<AUGMENTED_DIM, SCALAR>& state,
        const SCALAR& t,
        ct::core::ControlVector<CONTROL_DIM, SCALAR>& controlAction) override;

    ct::core::ControlMatrix<CONTROL_DIM, SCALAR> getDerivativeU0(
        const ct::core::StateVector<AUGMENTED_DIM, SCALAR>& state,
        const SCALAR time) override;

    ct::core::ControlMatrix<CONTROL_DIM, SCALAR> getDerivativeUf(
        const ct::core::StateVector<AUGMENTED_DIM, SCALAR>& state,
        const SCALAR time) override;

    void setController(std::shared_ptr<ct::core::Controller<STATE_DIM, CONTROL_DIM, SCALAR>> controller);

private:
    std::shared_ptr<ct::core::Controller<STATE_DIM, CONTROL_DIM, SCALAR>> controller_;
};

template <size_t STATE_DIM, size_t DIST_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class DisturbedSystem : public ct::core::ControlledSystem<STATE_DIM + DIST_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const size_t AUGMENTED_DIM = STATE_DIM + DIST_DIM;

    DisturbedSystem();

    DisturbedSystem(std::shared_ptr<ct::core::Controller<STATE_DIM, CONTROL_DIM, SCALAR>> controller);

    // Not overloaded. This is a completely new method.
    void setController(const std::shared_ptr<ct::core::Controller<STATE_DIM, CONTROL_DIM, SCALAR>>& controller);

    virtual void computeControlledDynamics(const ct::core::StateVector<AUGMENTED_DIM, SCALAR>& state,
        const SCALAR& t,
        const ct::core::ControlVector<CONTROL_DIM, SCALAR>& control,
        ct::core::StateVector<AUGMENTED_DIM, SCALAR>& derivative) = 0;
};

}  // optcon
}  // ct
