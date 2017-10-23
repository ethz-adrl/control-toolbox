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

#pragma once

#include "TermBase.hpp"

namespace ct {
namespace optcon {

/**
 * \ingroup CostFunction
 *
 * \brief A multiplicative term of type \f$ J = (x^T Q x) (u^T R u) \f$
 *
 * Probably this term is not very useful but we use it for testing.
 *
 * An example for using this term is given in \ref CostFunctionTest.cpp
 *
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL = double, typename SCALAR = SCALAR_EVAL>
class TermQuadMult : public TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Eigen::Matrix<SCALAR_EVAL, STATE_DIM, STATE_DIM> state_matrix_t;
    typedef Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, CONTROL_DIM> control_matrix_t;
    typedef Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, STATE_DIM> control_state_matrix_t;
    typedef Eigen::Matrix<SCALAR_EVAL, STATE_DIM, STATE_DIM> state_matrix_double_t;
    typedef Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, CONTROL_DIM> control_matrix_double_t;
    typedef Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, STATE_DIM> control_state_matrix_double_t;

    TermQuadMult();

    TermQuadMult(const state_matrix_t& Q, const control_matrix_t& R);

    TermQuadMult(const state_matrix_t& Q,
        const control_matrix_t& R,
        const core::StateVector<STATE_DIM, SCALAR_EVAL>& x_ref,
        core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u_ref);

    TermQuadMult(const TermQuadMult& arg);

    virtual ~TermQuadMult();

    TermQuadMult<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>* clone() const override;

    void setWeights(const state_matrix_double_t& Q, const control_matrix_double_t& R);

    void setStateAndControlReference(const core::StateVector<STATE_DIM>& x_ref,
        core::ControlVector<CONTROL_DIM>& u_ref);

    virtual SCALAR evaluate(const Eigen::Matrix<SCALAR, STATE_DIM, 1>& x,
        const Eigen::Matrix<SCALAR, CONTROL_DIM, 1>& u,
        const SCALAR& t) override;

    virtual ct::core::ADCGScalar evaluateCppadCg(const core::StateVector<STATE_DIM, ct::core::ADCGScalar>& x,
        const core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar>& u,
        ct::core::ADCGScalar t) override;

    core::StateVector<STATE_DIM, SCALAR_EVAL> stateDerivative(const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
        const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
        const SCALAR_EVAL& t) override;

    state_matrix_t stateSecondDerivative(const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
        const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
        const SCALAR_EVAL& t) override;

    core::ControlVector<CONTROL_DIM, SCALAR_EVAL> controlDerivative(const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
        const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
        const SCALAR_EVAL& t) override;

    control_matrix_t controlSecondDerivative(const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
        const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
        const SCALAR_EVAL& t) override;

    control_state_matrix_t stateControlDerivative(const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
        const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
        const SCALAR_EVAL& t) override;

    void loadConfigFile(const std::string& filename, const std::string& termName, bool verbose = false);


protected:
    template <typename SC>
    SC evalLocal(const Eigen::Matrix<SC, STATE_DIM, 1>& x, const Eigen::Matrix<SC, CONTROL_DIM, 1>& u, const SC& t);

    state_matrix_t Q_;
    control_matrix_t R_;

    core::StateVector<STATE_DIM, SCALAR_EVAL> x_ref_;
    core::ControlVector<CONTROL_DIM, SCALAR_EVAL> u_ref_;
};

}  // namespace optcon
}  // namespace ct
