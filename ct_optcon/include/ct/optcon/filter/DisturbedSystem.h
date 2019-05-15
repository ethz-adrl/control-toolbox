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
 * \brief Disturbed system augments the nominal system so that all the CT interfaces and dimensions are satisfied. What
 *        is done is basically augmenting the state with the assumed disturbance with specified dimensionality.
 *
 * @tparam STATE_DIM    nominal state dimensionality
 * @tparam DIST_DIM     dimensionality of the disturbance
 * @tparam CONTROL_DIM
 */
template <size_t STATE_DIM, size_t DIST_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class DisturbedSystem : public ct::core::ControlledSystem<STATE_DIM + DIST_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const size_t AUGMENTED_DIM = STATE_DIM + DIST_DIM;

    //! Constructor.
    DisturbedSystem();

    virtual ~DisturbedSystem() = default;

    //! Copy constructor.
    DisturbedSystem(std::shared_ptr<ct::core::Controller<STATE_DIM, CONTROL_DIM, SCALAR>> controller);

    // Not overloaded. This is a completely new method.
    void setController(const std::shared_ptr<ct::core::Controller<STATE_DIM, CONTROL_DIM, SCALAR>> controller);

    //! Implementation of the base computeControlledDynamics method.
    virtual void computeControlledDynamics(const ct::core::StateVector<AUGMENTED_DIM, SCALAR>& state,
        const SCALAR& t,
        const ct::core::ControlVector<CONTROL_DIM, SCALAR>& control,
        ct::core::StateVector<AUGMENTED_DIM, SCALAR>& derivative) = 0;
};

}  // namespace optcon
}  // namespace ct
