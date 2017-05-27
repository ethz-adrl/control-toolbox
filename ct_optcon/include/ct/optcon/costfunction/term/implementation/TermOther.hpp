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


template <size_t STATE_DIM, size_t CONTROL_DIM>
CppAD::AD<double> TermOther<STATE_DIM, CONTROL_DIM>::evaluate(const Eigen::Matrix<CppAD::AD<double>, STATE_DIM, 1> &x,
		const Eigen::Matrix<CppAD::AD<double>, CONTROL_DIM, 1> &u, const CppAD::AD<double>& t)
{
	Eigen::Matrix<CppAD::AD<double>, 1, 1> y_eigen = a_.transpose() * x + x.transpose() * x + u.transpose() * R_ * u; // FIXME: what is this?
	//Eigen::Matrix<CppAD::AD<double>, 1, 1> y_eigen = u.transpose() * u;

	CppAD::AD<double> y = y_eigen(0,0);
	return y;
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
void TermOther<STATE_DIM, CONTROL_DIM>::loadConfigFile(const std::string& filename, const std::string& termName, bool verbose) 
{
	// read in the file and put the valus in a_ and R_
	loadMatrixCF(filename,"a2", a_,termName);
	loadMatrixCF(filename,"R2", R_,termName);
	if(verbose){
		std::cout<<"Read a as a= \n"<<a_<<std::endl;
		std::cout<<"Read R as R= \n"<<R_<<std::endl;
	}
}
