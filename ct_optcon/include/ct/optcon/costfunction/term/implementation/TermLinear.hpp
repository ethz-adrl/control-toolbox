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

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR, typename SCALAR2>
TermLinear<STATE_DIM, CONTROL_DIM, SCALAR, SCALAR2>::TermLinear(const core::StateVector<STATE_DIM, SCALAR> a, core::ControlVector<CONTROL_DIM, SCALAR> b, const SCALAR c) : a_(a), b_(b), c_(c) {}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR, typename SCALAR2>
TermLinear<STATE_DIM, CONTROL_DIM, SCALAR, SCALAR2>::TermLinear(const TermLinear& arg) :
	TermBase<STATE_DIM, CONTROL_DIM, SCALAR, SCALAR2>(arg), a_(arg.a_), b_(arg.b_), c_(arg.c_)
{}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR, typename SCALAR2>
TermLinear<STATE_DIM, CONTROL_DIM, SCALAR, SCALAR2>::TermLinear() {}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR, typename SCALAR2>
TermLinear<STATE_DIM, CONTROL_DIM, SCALAR, SCALAR2>::~TermLinear() {}

// template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR, typename SCALAR2>
// template<typename SCALAR2>
// SCALAR2 TermLinear<STATE_DIM, CONTROL_DIM, SCALAR2>::evaluate(const Eigen::Matrix<SCALAR2, STATE_DIM, 1> &x, const Eigen::Matrix<SCALAR2, CONTROL_DIM, 1> &u, const SCALAR2& t)
// {
// 	Eigen::Matrix<SCALAR2, 1, 1> y_eigen = a_.transpose() * x + b_.transpose() * u;
// 	SCALAR2 y = y_eigen(0,0) + c_;
// 	return y;
// }

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR, typename SCALAR2>
core::StateVector<STATE_DIM, SCALAR> TermLinear<STATE_DIM, CONTROL_DIM, SCALAR, SCALAR2>::stateDerivative(const core::StateVector<STATE_DIM, SCALAR> &x, const core::ControlVector<CONTROL_DIM, SCALAR> &u, const SCALAR& t)
{
	return a_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR, typename SCALAR2>
typename TermLinear<STATE_DIM, CONTROL_DIM, SCALAR, SCALAR2>::state_matrix_t TermLinear<STATE_DIM, CONTROL_DIM, SCALAR, SCALAR2>::stateSecondDerivative(const core::StateVector<STATE_DIM, SCALAR> &x, const core::ControlVector<CONTROL_DIM, SCALAR> &u, const SCALAR& t)
{
	return state_matrix_t::Zero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR, typename SCALAR2>
core::ControlVector<CONTROL_DIM, SCALAR> TermLinear<STATE_DIM, CONTROL_DIM, SCALAR, SCALAR2>::controlDerivative(const core::StateVector<STATE_DIM, SCALAR> &x, const core::ControlVector<CONTROL_DIM, SCALAR> &u, const SCALAR& t)
{
	return b_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR, typename SCALAR2>
typename TermLinear<STATE_DIM, CONTROL_DIM, SCALAR, SCALAR2>::control_matrix_t TermLinear<STATE_DIM, CONTROL_DIM, SCALAR, SCALAR2>::controlSecondDerivative(const core::StateVector<STATE_DIM, SCALAR> &x, const core::ControlVector<CONTROL_DIM, SCALAR> &u, const SCALAR& t)
{
	return control_matrix_t::Zero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR, typename SCALAR2>
typename TermLinear<STATE_DIM, CONTROL_DIM, SCALAR, SCALAR2>::control_state_matrix_t TermLinear<STATE_DIM, CONTROL_DIM, SCALAR, SCALAR2>::stateControlDerivative(const core::StateVector<STATE_DIM, SCALAR> &x, const core::ControlVector<CONTROL_DIM, SCALAR> &u, const SCALAR& t)
{
	return control_state_matrix_t::Zero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR, typename SCALAR2>
void TermLinear<STATE_DIM, CONTROL_DIM, SCALAR, SCALAR2>::loadConfigFile(const std::string& filename, const std::string& termName, bool verbose)
{
       // read in the file and put the valus in a_ and b_
       loadMatrixCF(filename,"a", a_,termName);
       loadMatrixCF(filename,"b", b_,termName);
       if (verbose){
		   std::cout<<"Read a as a= \n"<<a_<<std::endl;
		   std::cout<<"Read b as b= \n"<<b_<<std::endl;
       }
}
