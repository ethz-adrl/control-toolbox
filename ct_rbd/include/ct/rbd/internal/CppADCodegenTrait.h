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

#ifndef INCLUDE_EXTERNAL_IIT_RBD_TRAITS_CPPADCODEGENTRAIT_H_
#define INCLUDE_EXTERNAL_IIT_RBD_TRAITS_CPPADCODEGENTRAIT_H_

#include <cppad/cg.hpp>
#include <cppad/cppad.hpp>
#include <cppad/example/cppad_eigen.hpp>


namespace ct {
namespace rbd {
namespace internal {

/*!
 * \brief Trait providing basic mathematical operations for CppAD Codegeneration datatypes
 */
class CppADCodegenTrait
{
public:
	typedef CppAD::AD<CppAD::cg::CG<double>> Scalar;

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
		Eigen::Matrix<Scalar, Dims, Dims> LU;
		Crout(A, LU);

		Eigen::Matrix<Scalar, Dims, 1> out;
		solveCrout(LU, b, out);

		return out;
	}


private:
	// Custom LU factorization
	template <typename T, int rowAndCol>  //only square matrices
	void static Crout(const Eigen::Matrix<T, rowAndCol, rowAndCol>& S, Eigen::Matrix<T, rowAndCol, rowAndCol>& D)
	{
		for (int k = 0; k < S.rows(); ++k)
		{
			for (int i = k; i < S.rows(); ++i)
			{
				T sum(0.);
				for (int p = 0; p < k; ++p)
					sum += D(i, p) * D(p, k);
				D(i, k) = S(i, k) - sum;  // not dividing by diagonals
			}
			for (int j = k + 1; j < S.rows(); ++j)
			{
				T sum(0.);
				for (int p = 0; p < k; ++p)
					sum += D(k, p) * D(p, j);
				D(k, j) = (S(k, j) - sum) / D(k, k);
			}
		}
	}

	// Custom LU solver
	template <typename T, int rowAndCol>
	void static solveCrout(const Eigen::Matrix<T, rowAndCol, rowAndCol>& LU,
		const Eigen::Matrix<T, rowAndCol, 1>& b,
		Eigen::Matrix<T, rowAndCol, 1>& x)
	{
		const int d = rowAndCol;
		T y[d];
		for (int i = 0; i < d; ++i)
		{
			T sum(0.0);
			for (int k = 0; k < i; ++k)
				sum += LU(i, k) * y[k];
			y[i] = (b(i) - sum) / LU(i, i);
		}
		for (int i = d - 1; i >= 0; --i)
		{
			T sum(0.);
			for (int k = i + 1; k < d; ++k)
				sum += LU(i, k) * x(k);
			x(i) = (y[i] - sum);  // not dividing by diagonals
		}
	}
};
}
}
}


#endif /* INCLUDE_EXTERNAL_IIT_RBD_TRAITS_CPPADCODEGENTRAIT_H_ */
