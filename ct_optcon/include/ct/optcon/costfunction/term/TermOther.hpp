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

#ifndef TERMOTHER_HPP_
#define TERMOTHER_HPP_

#include <cppad/example/cppad_eigen.hpp>

#include "../utility/utilities.hpp"
#include "TermBase.hpp"

namespace ct {
namespace optcon {

/**
 * \ingroup CostFunction
 *
 * \brief An example term using auto-diff
 *
 * Probably this term is not very useful but we use it for testing
 */
template <size_t STATE_DIM, size_t CONTROL_DIM>
class TermOther : public TermBase<STATE_DIM, CONTROL_DIM, CppAD::AD<double>, double > {

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	typedef CppAD::AD<double> SCALAR;

	CT_OPTCON_DEFINE_TERM_TYPES

	TermOther(const Eigen::Matrix<CppAD::AD<double>, STATE_DIM, 1> &a, const Eigen::Matrix<CppAD::AD<double>, CONTROL_DIM, CONTROL_DIM> &R) : a_(a), R_(R) {}

	TermOther() {}

	TermOther(const TermOther& arg):
		TermBase<STATE_DIM, CONTROL_DIM, CppAD::AD<double>, double > (arg)
	{
		a_ = arg.a_;
		R_ = arg.R_;
	}

	~TermOther() {}

	TermOther<STATE_DIM, CONTROL_DIM>* clone () const override {
		return new TermOther(*this);
	}

	CppAD::AD<double> evaluate(const Eigen::Matrix<CppAD::AD<double>, STATE_DIM, 1> &x, const Eigen::Matrix<CppAD::AD<double>, CONTROL_DIM, 1> &u, const CppAD::AD<double>& t) override;
	
	void loadConfigFile(const std::string& filename, const std::string& termName, bool verbose = false) override;  // virtual function for data loading

protected:
	Eigen::Matrix<CppAD::AD<double>, STATE_DIM, 1> a_;
	Eigen::Matrix<CppAD::AD<double>, CONTROL_DIM, CONTROL_DIM> R_;
};

#include "implementation/TermOther.hpp"

} // namespace optcon
} // namespace ct

#endif // TERMOTHER_HPP_
