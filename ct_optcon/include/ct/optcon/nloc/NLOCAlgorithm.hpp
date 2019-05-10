/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once


namespace ct {
namespace optcon {


template <size_t STATE_DIM,
    size_t CONTROL_DIM,
    size_t P_DIM,
    size_t V_DIM,
    typename SCALAR = double,
    bool CONTINUOUS = true>
class NLOCAlgorithm
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef ct::core::StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR> Policy_t;

    typedef NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS> Backend_t;
    typedef NLOptConSettings Settings_t;
    typedef SCALAR Scalar_t;

    NLOCAlgorithm(const std::shared_ptr<Backend_t>& backend) : backend_(backend) {}
    virtual ~NLOCAlgorithm() {}
    virtual void configure(const Settings_t& settings) = 0;

    virtual void prepareIteration() = 0;

    virtual bool finishIteration() = 0;

    virtual bool runIteration() = 0;

    virtual void setInitialGuess(const Policy_t& initialGuess) = 0;

    virtual void prepareMPCIteration() = 0;

    virtual bool finishMPCIteration() = 0;

protected:
    std::shared_ptr<Backend_t> backend_;
};
}  // namespace optcon
}  // namespace ct
