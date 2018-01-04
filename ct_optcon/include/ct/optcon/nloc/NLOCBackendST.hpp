/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/


#pragma once

#include "NLOCBackendBase.hpp"
#include <ct/optcon/solver/NLOptConSettings.hpp>

namespace ct {
namespace optcon {


/*!
 * NLOC Backend for Single-Threaded case
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR = double>
class NLOCBackendST : public NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR> Base;

    NLOCBackendST(const OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>& optConProblem, const NLOptConSettings& settings);

    NLOCBackendST(const OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>& optConProblem,
        const std::string& settingsFile,
        bool verbose = true,
        const std::string& ns = "alg");

    virtual ~NLOCBackendST();

protected:
    virtual void computeLQApproximation(size_t firstIndex, size_t lastIndex) override;

    virtual void rolloutShots(size_t firstIndex, size_t lastIndex) override;

    SCALAR performLineSearch() override;
};


}  // namespace optcon
}  // namespace ct
