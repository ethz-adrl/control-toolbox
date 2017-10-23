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

#include <cppad/cppad.hpp>
#include <cppad/example/cppad_eigen.hpp>

namespace ct {
namespace rbd {
namespace internal {

/*!
 * \brief Trait providing basic mathematical operations for CppAD datatypes
 */
struct CppADDoubleTrait
{
    typedef CppAD::AD<double> Scalar;

    inline static Scalar sin(const Scalar& x) { return CppAD::sin(x); }
    inline static Scalar cos(const Scalar& x) { return CppAD::cos(x); }
    inline static Scalar tan(const Scalar& x) { return CppAD::tan(x); }
    inline static Scalar sinh(const Scalar& x) { return CppAD::sinh(x); }
    inline static Scalar cosh(const Scalar& x) { return CppAD::cosh(x); }
    inline static Scalar tanh(const Scalar& x) { return CppAD::tanh(x); }
    inline static Scalar exp(const Scalar& x) { return CppAD::exp(x); }
    inline static Scalar fabs(const Scalar& x) { return CppAD::fabs(x); }
    template <int Dims>
    inline static Eigen::Matrix<Scalar, Dims, 1> solve(const Eigen::Matrix<Scalar, Dims, Dims>& A,
        const Eigen::Matrix<Scalar, Dims, 1>& b)
    {
        return A.inverse() * b;
    }
};
}
}
}
