/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

#include <ct/optcon/solver/NLOptConSettings.hpp>
#include <ct/optcon/nloc/NLOCAlgorithm.hpp>

namespace ct {
namespace optcon {


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR = double>
class iLQR : public NLOCAlgorithm<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const size_t STATE_D = STATE_DIM;
    static const size_t CONTROL_D = CONTROL_DIM;

    typedef ct::core::StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR> Policy_t;
    typedef NLOptConSettings Settings_t;
    typedef SCALAR Scalar_t;

    typedef NLOCAlgorithm<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR> BASE;
    typedef NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR> Backend_t;

    //! constructor
    iLQR(std::shared_ptr<Backend_t>& backend_, const Settings_t& settings);

    //! destructor
    virtual ~iLQR();

    //! configure the solver
    virtual void configure(const Settings_t& settings) override;

    //! set an initial guess
    virtual void setInitialGuess(const Policy_t& initialGuess) override;


    //! runIteration combines prepareIteration and finishIteration
    /*!
	 * For iLQR the separation between prepareIteration and finishIteration would actually not be necessary
	 * @return
	 */
    virtual bool runIteration() override;


    /*!
	 * for iLQR, as it is a purely sequential approach, we cannot prepare anything prior to solving,
	 */
    virtual void prepareIteration() override;


    /*!
	 * for iLQR, finishIteration contains the whole main iLQR iteration.
	 * @return
	 */
    virtual bool finishIteration() override;


    /*!
	 * for iLQR, as it is a purely sequential approach, we cannot prepare anything prior to solving,
	 */
    virtual void prepareMPCIteration() override;


    /*!
	 * for iLQR, finishIteration contains the whole main iLQR iteration.
	 * @return
	 */
    virtual bool finishMPCIteration() override;
};

}  // namespace optcon
}  // namespace ct
