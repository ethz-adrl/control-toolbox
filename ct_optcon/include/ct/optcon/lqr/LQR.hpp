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

#include "riccati/CARE.hpp"

#ifdef USE_MATLAB_CPP_INTERFACE
#include <matlabCppInterface/Engine.hpp>
#endif

namespace ct {
namespace optcon {

/*!
 * \ingroup LQR
 *
 * \brief continuous-time infinite-horizon LQR
 *
 * Implements continous-time infinite-horizon LQR.
 * Resulting feedback law will take the form
 * \f[
 * u_{fb} = -K \cdot (x - x_{ref})
 * \f]
 *
 * @tparam STATE_DIM
 * @tparam CONTROL_DIM
 */
template <size_t STATE_DIM, size_t CONTROL_DIM>
class LQR
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Eigen::Matrix<double, STATE_DIM, STATE_DIM> state_matrix_t;
    typedef Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> control_matrix_t;
    typedef Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> control_state_matrix_t;
    typedef Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> control_gain_matrix_t;
    typedef Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> control_feedback_t;

    //! design the infinite-horizon LQR controller.
    /*!
	 * @param Q state-weighting matrix
	 * @param R control input weighting matrix
	 * @param A linear system dynamics matrix A
	 * @param B linear system dynamics matrix B
	 * @param K control feedback matrix K (to be designed)
	 * @param RisDiagonal set to true if R is a diagonal matrix (efficiency boost)
	 * @param solveRiccatiIteratively
	 * 	use closed-form solution of the infinite-horizon Riccati Equation
	 * @return success
	 */
    bool compute(const state_matrix_t& Q,
        const control_matrix_t& R,
        const state_matrix_t& A,
        const control_gain_matrix_t& B,
        control_feedback_t& K,
        bool RisDiagonal = false,
        bool solveRiccatiIteratively = false);

#ifdef USE_MATLAB_CPP_INTERFACE
    //! design the LQR controller in MATLAB
    /*!
	 * Note that this controller should be exactly the same
	 */
    bool computeMatlab(const state_matrix_t& Q,
        const control_matrix_t& R,
        const state_matrix_t& A,
        const control_gain_matrix_t& B,
        control_feedback_t& K,
        bool checkControllability = false);
#endif  //USE_MATLAB_CPP_INTERFACE


private:
    CARE<STATE_DIM, CONTROL_DIM> care_;  // continuous-time algebraic riccati equation

#ifdef USE_MATLAB_CPP_INTERFACE
    matlab::Engine matlabEngine_;
#endif
};


}  // namespace optcon
}  // namespace ct
