/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "DisturbedSystem.h"

namespace ct {
namespace optcon {

/**
 * @brief Implementation of an input disturbed system where, the dimension of the disturbance is equal to the
 * dimension of the control input, thus DIST_DIM = CONTROL_DIM. This is a special case, however it occurs often 
 * and is convenient to have as separate class.
 * 
 * The new state vector has the form x_aug = [x_system + disturbance];
 * 
 * @note this system should not be for simulation, but for filtering. It is not suitable for simulation since the
 * disturbance state gets mapped back to the control input.
 * 
 * @tparam STATE_DIM state dimension
 * @tparam CONTROL_DIM 
 * @tparam double 
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class InputDisturbedSystem : public ct::optcon::DisturbedSystem<STATE_DIM, CONTROL_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // the dimension of the augmented state used in the filter
    static const size_t AUGMENTED_STATE_DIM = STATE_DIM + CONTROL_DIM;

    using Base = typename ct::optcon::DisturbedSystem<STATE_DIM, CONTROL_DIM, CONTROL_DIM, SCALAR>;

    /**
     * @brief Construct a new Input Disturbed System object
     * 
     * @param sys the system that is subject to a disturbance
     */
    InputDisturbedSystem(std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>> sys);

    //! copy constructor
    InputDisturbedSystem(const InputDisturbedSystem& other);

    //! deep cloning
    InputDisturbedSystem* clone() const override;

    /**
     * @brief compute the dynamics (the left-hand-side of the dynamics equation) for the disturbance-augmented system
     * 
     * @param state the augmented state
     * @param t the current time
     * @param control the nominal control (non-perturbed)
     * @param derivative the left-hand side of the augmented system
     */
    void computeControlledDynamics(const ct::core::StateVector<AUGMENTED_STATE_DIM, SCALAR>& state,
        const SCALAR& t,
        const ct::core::ControlVector<CONTROL_DIM, SCALAR>& control,
        ct::core::StateVector<AUGMENTED_STATE_DIM, SCALAR>& derivative) override;

private:
    // the nominal system (the one we are trying to control)
    std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>> system_;
};

}  // namespace optcon
}  // namespace ct
