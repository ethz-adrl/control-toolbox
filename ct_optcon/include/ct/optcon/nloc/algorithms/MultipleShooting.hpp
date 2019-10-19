/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

#include <ct/optcon/solver/NLOptConSettings.hpp>
#include <ct/optcon/nloc/NLOCAlgorithm.hpp>

namespace ct {
namespace optcon {


template <size_t STATE_DIM,
    size_t CONTROL_DIM,
    size_t P_DIM,
    size_t V_DIM,
    typename SCALAR = double,
    bool CONTINUOUS = true>
class MultipleShooting final : public NLOCAlgorithm<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const size_t STATE_D = STATE_DIM;
    static const size_t CONTROL_D = CONTROL_DIM;

    typedef NLOCAlgorithm<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS> Base;

    typedef typename Base::Policy_t Policy_t;
    typedef typename Base::Settings_t Settings_t;
    typedef typename Base::Backend_t Backend_t;

    typedef SCALAR Scalar_t;

    //! constructor
    MultipleShooting(std::shared_ptr<Backend_t>& backend_, const Settings_t& settings);

    //! destructor
    virtual ~MultipleShooting() = default;

    //! configure the solver
    virtual void configure(const Settings_t& settings) override;

    //! set an initial guess
    virtual void setInitialGuess(const Policy_t& initialGuess) override;

    //! runIteration combines prepareIteration and finishIteration
    /*!
     * @return foundBetter (false if converged)
     */
    virtual bool runIteration() override;


    /*!
     * - linearize dynamics for the stages 1 to N-1
     * - quadratize cost for stages 1 to N-1
     */
    virtual void prepareIteration() override;


    //! finish iteration for unconstrained MultipleShooting
    /*!
     * - linearize dynamcs for the first stage
     * - quadratize cost for the first stage
     * @return
     */
    virtual bool finishIteration() override;


    //! prepare iteration, dedicated to MPC.
    /*!
     * requirements: no line-search, end with update-step of controls and state, no rollout after update steps.
     * Therefore: rollout->linearize->solve
     */
    virtual void prepareMPCIteration() override;


    //! finish iteration, dedicated to MPC
    virtual bool finishMPCIteration() override;
};

}  // namespace optcon
}  // namespace ct
