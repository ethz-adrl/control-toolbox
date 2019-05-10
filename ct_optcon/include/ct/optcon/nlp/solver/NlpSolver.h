/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/optcon/nlp/Nlp.h>
#include "NlpSolverSettings.h"

namespace ct {
namespace optcon {
namespace tpl {

/**
 * @ingroup    NLP
 *
 * @brief      Abstract base class for the NLP solvers
 */
template <typename SCALAR>
class NlpSolver
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
	 * @brief      Default constructor
	 */
    NlpSolver() : isInitialized_(false) {}
    /**
	 * @brief      Custom Constructor 1
	 *
	 * @param[in]  nlp   The nlp
	 */
    NlpSolver(std::shared_ptr<Nlp<SCALAR>> nlp) : nlp_(nlp) {}
    /**
	 * @brief      Custom constructor 2
	 *
	 * @param[in]  nlp       The nlp
	 * @param[in]  settings  Custom user settings
	 */
    NlpSolver(std::shared_ptr<Nlp<SCALAR>> nlp, NlpSolverSettings settings)
        : nlp_(nlp), settings_(settings), isInitialized_(false)
    {
    }

    /**
	 * @brief      Destructor
	 */
    virtual ~NlpSolver() = default;
    /**
	 * @brief      Configures the solver with new settings
	 *
	 * @param[in]  settings  The nlp solver settings
	 */
    void configure(const NlpSolverSettings& settings)
    {
        settings_ = settings;
        configureDerived(settings);
    }

    /**
	 * @brief      Forwards the settings to the corresponding nlp solver
	 *
	 * @param[in]  settings  The nlp settings
	 */
    virtual void configureDerived(const NlpSolverSettings& settings) = 0;

    /**
	 * @brief      Solves the nlp
	 *
	 * @return     Returns true of solve succeeded
	 */
    virtual bool solve() = 0;

    /**
	 * @brief      Prepares the solver for a warmstarting scenario with
	 *             available (good) initial guess
	 *
	 * @param[in]  maxIterations  Specifies the maximum number of nlp iteration
	 *                            the user is willing to spend
	 */
    virtual void prepareWarmStart(const size_t maxIterations) = 0;

    bool isInitialized() { return isInitialized_; }
protected:
    std::shared_ptr<Nlp<SCALAR>> nlp_; /*!< The non linear program*/
    NlpSolverSettings settings_;       /*!< The nlp settings */
    bool isInitialized_;               /*!< Indicates whether the solver is initialized */
};
}

using NlpSolver = tpl::NlpSolver<double>;

}  // namespace optcon
}  // namespace ct
