/***********************************************************************************
Copyright (c) 2016, Agile & Dexterous Robotics Lab, ETH ZURICH. All rights reserved.

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

#ifndef CT_OPTCON_NLP_SOLVER_NLPSOLVER_H_
#define CT_OPTCON_NLP_SOLVER_NLPSOLVER_H_

#include <ct/optcon/nlp/Nlp.h>
#include "NlpSolverSettings.h"

namespace ct {
namespace optcon {

/**
 * @ingroup    NLP
 *
 * @brief      Abstract base class for the NLP solvers
 */
class NlpSolver
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/**
	 * @brief      Default constructor
	 */
	NlpSolver() :
	isInitialized_(false)
	{}

	/**
	 * @brief      Custom Constructor 1
	 *
	 * @param[in]  nlp   The nlp
	 */
	NlpSolver(std::shared_ptr<Nlp> nlp) 
	: 
		nlp_(nlp)
	{}

	/**
	 * @brief      Custom constructor 2
	 *
	 * @param[in]  nlp       The nlp
	 * @param[in]  settings  Custom user settings
	 */
	NlpSolver(std::shared_ptr<Nlp> nlp, NlpSolverSettings settings) 
	: 
		nlp_(nlp),
		settings_(settings),
		isInitialized_(false)
	{}

	/**
	 * @brief      Destructor
	 */
	virtual ~NlpSolver() {}

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

protected:
	std::shared_ptr<Nlp> nlp_; /*!< The non linear program*/
	NlpSolverSettings settings_; /*!< The nlp settings */
	bool isInitialized_; /*!< Indicates whether the solver is initialized */
};

} // namespace ct
} // namespace optcon



#endif //CT_OPTCON_NLP_SOLVER_NLPSOLVER_H_