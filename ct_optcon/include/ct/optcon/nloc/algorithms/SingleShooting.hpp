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
class SingleShooting final : public NLOCAlgorithm<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>
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
    SingleShooting(std::shared_ptr<Backend_t>& backend_, const Settings_t& settings);

    //! destructor
    virtual ~SingleShooting() = default;

    //! configure the solver
    virtual void configure(const Settings_t& settings) override;

    //! set an initial guess
    virtual void setInitialGuess(const Policy_t& initialGuess) override;


    //! runIteration combines prepareIteration and finishIteration
    /*!
     * For SingleShooting the separation between prepareIteration and finishIteration would actually not be necessary
     * @return
     */
    virtual bool runIteration() override;


    /*!
     * for SingleShooting, as it is a purely sequential approach, we cannot prepare anything prior to solving,
     */
    virtual void prepareIteration() override;


    /*!
     * for SingleShooting, finishIteration contains the whole main SingleShooting iteration.
     * @return
     */
    virtual bool finishIteration() override;


    /*!
     * for SingleShooting, as it is a purely sequential approach, we cannot prepare anything prior to solving,
     */
    virtual void prepareMPCIteration() override;


    /*!
     * for SingleShooting, finishIteration contains the whole main SingleShooting iteration.
     * @return
     */
    virtual bool finishMPCIteration() override;
};

}  // namespace optcon
}  // namespace ct
