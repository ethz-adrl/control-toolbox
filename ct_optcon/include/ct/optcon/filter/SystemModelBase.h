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
 * \brief System model is an interface that encapsulates the integrator to be able to propagate the system, but is also
 *        able to compute derivatives w.r.t. both state and noise.
 *
 * @tparam STATE_DIM
 * @tparam CONTROL_DIM
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class SystemModelBase
{
public:
    using state_vector_t = ct::core::StateVector<STATE_DIM, SCALAR>;
    using state_matrix_t = ct::core::StateMatrix<STATE_DIM, SCALAR>;
    using control_vector_t = ct::core::ControlVector<CONTROL_DIM, SCALAR>;
    using Time_t = SCALAR;

    //! Virtual destructor.
    virtual ~SystemModelBase() = default;

    //! Propagates the system giving the next state as output.
    virtual state_vector_t computeDynamics(const state_vector_t& state,
        const control_vector_t& control,
        const Time_t dt,
        Time_t t) = 0;

    //! Computes the derivative w.r.t state.
    virtual state_matrix_t computeDerivativeState(const state_vector_t& state,
        const control_vector_t& control,
        const Time_t dt,
        Time_t t) = 0;

    //! Computes the derivative w.r.t noise.
    virtual state_matrix_t computeDerivativeNoise(const state_vector_t& state,
        const control_vector_t& control,
        const Time_t dt,
        Time_t t) = 0;
};

}  // namespace optcon
}  // namespace ct
