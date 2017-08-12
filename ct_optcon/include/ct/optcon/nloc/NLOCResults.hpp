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


#ifndef CT_OPTCON_INCLUDE_CT_OPTCON_NLOC_NLOCRESULTS_HPP_
#define CT_OPTCON_INCLUDE_CT_OPTCON_NLOC_NLOCRESULTS_HPP_

/*!
 * to do log every xth iter
 *
 * \todo also log settings here
 */
template <typename SCALAR = double>
struct SummaryAllIterations
{
	//! log of the iterations
	std::vector<size_t> iterations_;

	//! different overall defect norms
	std::vector<SCALAR> defect_l1_norms;
	std::vector<SCALAR> defect_l2_norms;

	//! step update norms
	std::vector<SCALAR> lx_norms;
	std::vector<SCALAR> lu_norms;

	//! costs and merits
	std::vector<SCALAR> intermediateCosts;
	std::vector<SCALAR> finalCosts;
	std::vector<SCALAR> totalCosts;
	std::vector<SCALAR> merits;

	//! step size
	std::vector<SCALAR> stepSizes;

	template <int numPrecision = 12>
	void printSummaryLastIteration()
	{
		std::cout<<"NLOC Summary of iteration "  << iterations_.back() << std::endl;
		std::cout<<"=============================="<< std::endl;

		std::cout<<std::setprecision(numPrecision) << "interm. cost:\t" << intermediateCosts.back() << std::endl;
		std::cout<<std::setprecision(numPrecision) << "final cost:\t" << finalCosts.back() << std::endl;
		std::cout<<std::setprecision(numPrecision) << "total cost:\t" << totalCosts.back() << std::endl;
		std::cout<<std::setprecision(numPrecision) << "total merit:\t" << merits.back() << std::endl;
		std::cout<<std::setprecision(numPrecision) << "tot. defect L1:\t" << defect_l1_norms.back() << std::endl;
		std::cout<<std::setprecision(numPrecision) << "tot. defect L2:\t" << defect_l2_norms.back() << std::endl;
		std::cout<<std::setprecision(numPrecision) << "total lx norm:\t" << lx_norms.back() << std::endl;
		std::cout<<std::setprecision(numPrecision) << "total lu norm:\t" << lu_norms.back() << std::endl;
		std::cout<<std::setprecision(numPrecision) << "step-size:\t" << stepSizes.back() << std::endl;
		std::cout<<"                   ===========" << std::endl;
		std::cout<<std::endl;
	}

};


#endif /* CT_OPTCON_INCLUDE_CT_OPTCON_NLOC_NLOCRESULTS_HPP_ */
