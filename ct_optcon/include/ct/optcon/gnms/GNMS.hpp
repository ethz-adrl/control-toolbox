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

#ifndef GNMS_HPP_
#define GNMS_HPP_

#include "GNMSBase.hpp"

namespace ct{
namespace optcon{

//!  Single-Threaded Implementation of GNMS/SLQ
/*!
 * \ingroup GNMS
 *
 * C++ implementation of GNMS. In fact, this currently implements iLQR.
 *
 * The implementation and naming is based on the reference below. In general, the code follows this convention:
 * X  <- Matrix (upper-case in paper)
 * xv <- vector (lower-case bold in paper)
 * x  <- scalar (lower-case in paper)
 *
 * References:
 * Sideris, Athanasios, and James E. Bobrow. "An efficient sequential linear quadratic algorithm for solving nonlinear optimal control problems."
 * Proceedings of the American Control Conference, 2005, pp. 2275-2280
 *
 * Todorov, E.; Weiwei Li, "A generalized iterative LQG method for locally-optimal feedback control of constrained nonlinear stochastic systems,"
 * Proceedings of the American Control Conference, 2005, pp.300-306
 *
*/
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class GNMS : public GNMSBase<STATE_DIM, CONTROL_DIM, SCALAR> {

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef GNMSBase<STATE_DIM, CONTROL_DIM, SCALAR> Base;

	typedef typename Base::Policy_t Policy_t;
	typedef typename Base::Settings_t Settings_t;

    //! GNMS constructor.
    /*!
      Sets up GNMS. Dynamics, derivatives of the dynamics as well as the cost function have to be provided.
      You should pass pointers to instances of classes here that derive from the dynamics, derivatives and costFunction base classes

    */
	GNMS(const OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>& optConProblem,
			const GNMSSettings& settings) :
		Base(optConProblem, settings)
	{

	}

	GNMS(const OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>& optConProblem,
		 const std::string& settingsFile,
		 bool verbose = true,
		 const std::string& ns = "ilqg") :
		Base(optConProblem, settingsFile, verbose, ns)
	{
	}




private:
	void createLQProblem() override;

	void backwardPass() override;

	void computeQuadraticCostsAroundTrajectory() override;

	void computeLinearizedDynamicsAroundTrajectory() override;

	void initializeShots() override;

	void updateShots() override;

	void computeDefects() override;

};


#include "implementation/GNMS.hpp"

} // namespace optcon
} // namespace ct


#endif /* GNMS_HPP_ */
