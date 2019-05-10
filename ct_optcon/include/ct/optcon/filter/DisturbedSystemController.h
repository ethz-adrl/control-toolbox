/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

/*!
 * \ingroup Filter
 *
 * \brief Disturbed controller allows us to augment the controller so that all the CT interfaces and dimensions are
 *        satisfied. Augmenting is done in such a way that the nominal controller is wrapped and all the nominal states
 *        are controlled in the same way as before augmenting the state. The augmented state (the disturbance part) is
 *        assumed constant, thus the computed derivates of that part of the state are set to zero.
 *
 * @tparam STATE_DIM    nominal state dimensionality
 * @tparam DIST_DIM     dimensionality of the disturbance
 * @tparam CONTROL_DIM
 */
template <size_t STATE_DIM, size_t DIST_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class DisturbedSystemController : public ct::core::Controller<STATE_DIM + DIST_DIM, CONTROL_DIM, SCALAR>
{
public:
    static const size_t AUGMENTED_DIM = STATE_DIM + DIST_DIM;

    //! Constructor. Takes in the nominal controller
    DisturbedSystemController(
        std::shared_ptr<ct::core::Controller<STATE_DIM, CONTROL_DIM, SCALAR>> controller = nullptr);

    //! Copy constructor.
    DisturbedSystemController(const DisturbedSystemController& other);

    //! Clone method.
    DisturbedSystemController* clone() const override;

    //! Implementation of the base computeControl method.
    void computeControl(const ct::core::StateVector<AUGMENTED_DIM, SCALAR>& state,
        const SCALAR& t,
        ct::core::ControlVector<CONTROL_DIM, SCALAR>& controlAction) override;

    //! Implementation of the base getDerivativeU0 method.
    ct::core::ControlMatrix<CONTROL_DIM, SCALAR> getDerivativeU0(
        const ct::core::StateVector<AUGMENTED_DIM, SCALAR>& state,
        const SCALAR time) override;

    //! Implementation of the base getDerivativeUf method.
    ct::core::ControlMatrix<CONTROL_DIM, SCALAR> getDerivativeUf(
        const ct::core::StateVector<AUGMENTED_DIM, SCALAR>& state,
        const SCALAR time) override;

    //! Sets the nominal controller.
    void setController(std::shared_ptr<ct::core::Controller<STATE_DIM, CONTROL_DIM, SCALAR>> controller);

private:
    std::shared_ptr<ct::core::Controller<STATE_DIM, CONTROL_DIM, SCALAR>> controller_;  //! Nominal controller.
};

}  // namespace optcon
}  // namespace ct
