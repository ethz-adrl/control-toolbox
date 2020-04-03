/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "TermBase.hpp"

namespace ct {
namespace optcon {

/**
 * \ingroup CostFunction
 *
 * \brief A basic quadratic term of type \f$ J = x^T Q x + u^T R u \f$
 *
 *  An example for using this term is given in \ref CostFunctionTest.cpp
 *
 */
template <typename MANIFOLD, size_t CONTROL_DIM, typename AD_MANIFOLD = MANIFOLD>
class TermQuadratic : public TermBase<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static constexpr size_t STATE_DIM = MANIFOLD::TangentDim;

    using Base = TermBase<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>;

    using SCALAR = typename Base::SCALAR;
    using SCALAR_EVAL = typename Base::SCALAR_EVAL;

    using state_matrix_t = typename Base::state_matrix_t;
    using control_matrix_t = typename Base::control_matrix_t;
    using control_state_matrix_t = typename Base::control_state_matrix_t;

    TermQuadratic();

    TermQuadratic(const state_matrix_t& Q, const control_matrix_t& R);

    TermQuadratic(const state_matrix_t& Q,
        const control_matrix_t& R,
        const MANIFOLD& x_ref,
        const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u_ref);

    TermQuadratic(const std::string& configFile, const std::string& termName, bool verbose = false);

    TermQuadratic(const TermQuadratic& arg);

    virtual ~TermQuadratic();

    virtual TermQuadratic<MANIFOLD, CONTROL_DIM, AD_MANIFOLD>* clone() const override;

    void setWeights(const state_matrix_t& Q, const control_matrix_t& R);

    const state_matrix_t& getStateWeight() const;

    state_matrix_t& getStateWeight();

    const control_matrix_t& getControlWeight() const;

    control_matrix_t& getControlWeight();

    void setStateAndControlReference(const MANIFOLD& x_ref, const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u_ref);

    virtual SCALAR evaluate(const AD_MANIFOLD& x,
        const ct::core::ControlVector<CONTROL_DIM, SCALAR>& u,
        const SCALAR& t) override;

    core::StateVector<STATE_DIM, SCALAR_EVAL> stateDerivative(const MANIFOLD& x,
        const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
        const SCALAR_EVAL& t) override;

    state_matrix_t stateSecondDerivative(const MANIFOLD& x,
        const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
        const SCALAR_EVAL& t) override;

    core::ControlVector<CONTROL_DIM, SCALAR_EVAL> controlDerivative(const MANIFOLD& x,
        const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
        const SCALAR_EVAL& t) override;

    control_matrix_t controlSecondDerivative(const MANIFOLD& x,
        const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
        const SCALAR_EVAL& t) override;

    control_state_matrix_t stateControlDerivative(const MANIFOLD& x,
        const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
        const SCALAR_EVAL& t) override;

    virtual void loadConfigFile(const std::string& filename,
        const std::string& termName,
        bool verbose = false) override;

    virtual void updateReferenceState(const MANIFOLD& newRefState) override;

    virtual void updateReferenceControl(
        const ct::core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u) override;

    virtual MANIFOLD getReferenceState() const override;

protected:
    state_matrix_t Q_;
    control_matrix_t R_;

    core::StateVector<STATE_DIM, SCALAR_EVAL> x_ref_;
    core::ControlVector<CONTROL_DIM, SCALAR_EVAL> u_ref_;
};


}  // namespace optcon
}  // namespace ct
