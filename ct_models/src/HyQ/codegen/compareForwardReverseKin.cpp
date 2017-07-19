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

#include <ct/rbd/rbd.h>

#include <ct/models/HyQ/HyQ.h>
#include <ct/models/HyQ/codegen/HyQForwardKinJacForward.h>
#include <ct/models/HyQ/codegen/HyQForwardKinJacReverse.h>

#include "helperFunctions.h"

using namespace ct::models::HyQ;

void timing()
{
	std::cout << "Timing forward kinematics "<<std::endl;
	static const size_t nTests = 10000;

	const size_t IN_DIM = state_dim;
	const size_t OUT_DIM = nEE*6;
	typedef Eigen::Matrix<double, IN_DIM, 1> X;
	typedef Eigen::Matrix<double, OUT_DIM, IN_DIM> Jac;

	std::vector<X, Eigen::aligned_allocator<X>> x(nTests);

	std::vector<Jac, Eigen::aligned_allocator<Jac>> forward(nTests);
	std::vector<Jac, Eigen::aligned_allocator<Jac>> reverse(nTests);
	std::vector<Jac, Eigen::aligned_allocator<Jac>> numDiff(nTests);

	HyQForwardKinJacForward forwardJac;
	HyQForwardKinJacReverse reverseJac;
	ct::core::DerivativesNumDiff<IN_DIM, OUT_DIM>::Function idFun = ct::models::HyQ::hyqForwardKinematics<double>;
	ct::core::DerivativesNumDiff<IN_DIM, OUT_DIM> numDiffJac(idFun);

	std::cout << "input dim: "<<IN_DIM<<", output dim: "<<OUT_DIM<<std::endl;

	std::cout << "running "<<nTests<<" tests"<<std::endl;
	for (size_t i=0; i<nTests; i++)
	{
		x[i].setRandom();
	}

	auto start = std::chrono::high_resolution_clock::now();
	for (size_t i=0; i<nTests; i++)
	{
		forward[i] = forwardJac.jacobian(x[i]);
	}
	auto end = std::chrono::high_resolution_clock::now();
	auto diff = end - start;
	size_t msTotal = std::chrono::duration <double, std::micro> (diff).count();
	std::cout << "forwardA: " << msTotal/1000.0 << " ms. Average: " << msTotal/double(nTests)/1000.0 << " ms" << std::endl;

	start = std::chrono::high_resolution_clock::now();
	for (size_t i=0; i<nTests; i++)
	{
		reverse[i] = reverseJac.jacobian(x[i]);
	}
	end = std::chrono::high_resolution_clock::now();
	diff = end - start;
	msTotal = std::chrono::duration <double, std::micro> (diff).count();
	std::cout << "reverseA: " << msTotal/1000.0 << " ms. Average: " << msTotal/double(nTests)/1000.0 << " ms" << std::endl;

	start = std::chrono::high_resolution_clock::now();
	for (size_t i=0; i<nTests; i++)
	{
		numDiff[i] = numDiffJac.jacobian(x[i]);
	}
	end = std::chrono::high_resolution_clock::now();
	diff = end - start;
	msTotal = std::chrono::duration <double, std::micro> (diff).count();
	std::cout << "numDiffA: " << msTotal/1000.0 << " ms. Average: " << msTotal/double(nTests)/1000.0 << " ms" << std::endl;



	bool failed = false;
	for (size_t i=0; i<nTests; i++)
	{
		if (!forward[i].isApprox(numDiff[i], 1e-5))
		{
			std::cout << "Forward and NumDiff not similar" << std::endl;
			std::cout << "forward: "<<std::endl << forward[i] << std::endl;
			std::cout << "numDiff: "<<std::endl << numDiff[i] << std::endl<< std::endl<< std::endl;
			failed = true;
		}
		if (!forward[i].isApprox(reverse[i], 1e-12))
		{
			std::cout << "Forward and reverse not similar" << std::endl;
			std::cout << "forward: "<<std::endl << forward[i] << std::endl;
			std::cout << "reverse: "<<std::endl << reverse[i] << std::endl<< std::endl<< std::endl;
			failed = true;
		}
		if (failed)
		{
			std::cout << "test failed, aborting"<<std::endl;
			break;
		}
	}
}


int main (int argc, char* argv[])
{
	timing();
	return 0;
}



