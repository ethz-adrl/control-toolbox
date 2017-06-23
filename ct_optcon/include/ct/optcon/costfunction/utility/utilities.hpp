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

#ifndef CT_OPTCON_TERMS_UTILITIES_HPP_
#define CT_OPTCON_TERMS_UTILITIES_HPP_

#include <iostream>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>

#include <ct/optcon/costfunction/term/TermBase.hpp>

#include "../CostFunction.hpp"

namespace ct {
namespace optcon {

template <typename SCALAR>
void loadScalarCF(const std::string& filename, const std::string& scalarName, SCALAR& scalar, const std::string& termName="")
{
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(filename, pt);

    scalar = pt.get<SCALAR>(termName +".weights." + scalarName);
}

template <typename SCALAR>
void loadScalarOptionalCF(const std::string& filename, const std::string& scalarName, SCALAR& scalar, const std::string& termName, const SCALAR& defaultValue)
{
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(filename, pt);

    scalar = pt.get<SCALAR>(termName +".weights." + scalarName, defaultValue);
}

template <typename SCALAR, int ROW, int COL>
void loadMatrixCF(const std::string& filename, const std::string& matrixName, Eigen::Matrix<SCALAR, ROW, COL> &matrix, const std::string& termName="")
{
	size_t rows = matrix.rows();
	size_t cols = matrix.cols();

	boost::property_tree::ptree pt;
	boost::property_tree::read_info(filename, pt);

	double scaling = pt.get<double>(termName +".weights." + matrixName + ".scaling", 1);

	for (size_t i=0; i<rows; i++)
	{
		for (size_t j=0; j<cols; j++)
		{
			if(termName==""){
				matrix(i,j) = scaling*pt.get<double>(matrixName + "." + "(" +std::to_string(i) + "," + std::to_string(j) + ")" , 0.0);
			}
			else{
				matrix(i,j) = scaling*pt.get<double>(termName +".weights." + matrixName + "." + "(" +std::to_string(i) + "," + std::to_string(j) + ")" , 0.0);
			}
		}
	}
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR, typename TIME_SCALAR, typename costFuncType>
void addTerm (const std::string& filename, std::string& currentTerm, int currentTermType, std::shared_ptr< TermBase<STATE_DIM, CONTROL_DIM, SCALAR, TIME_SCALAR> > term, costFuncType *costFunc, bool verbose = false)
{
	switch (currentTermType){
	case 0:
		costFunc->addIntermediateTerm(term, verbose);
		break;
	case 1:
		costFunc->addFinalTerm(term, verbose);
		break;
	default:
		if(verbose){
			std::cout<<"error code 1 => term type other than term0 and term1 encountered"<<std::endl;
		}
		BOOST_PROPERTY_TREE_THROW(boost::property_tree::info_parser::info_parser_error("read error code = ", "", 1));//error code 1 => term type otherthan term0 and term1 encountered
		break;
	}
	term->loadTimeActivation(filename, currentTerm, verbose);
	term->loadConfigFile(filename, currentTerm, verbose);
	std::cout << "Successfully loaded term" << std::endl;
}

} // namespace optcon
} // namepsace ct

#endif //CT_OPTCON_TERMS_UTILITIES_HPP_
