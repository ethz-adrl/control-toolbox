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

#ifndef CT_CORE_LINEARFUNCTIONS_H_
#define CT_CORE_LINEARFUNCTIONS_H_

#include <ct/core/types/trajectories/DiscreteArray.h>
#include <ct/core/types/trajectories/TimeArray.h>

namespace ct{
namespace core{

template <int STATE_DIM, int CONTROL_DIM, class SCALAR = double>
class LinearFunctionMIMO {

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	void swap(LinearFunctionMIMO& arg){
		uff_.swap(arg.uff_);
		deltaUff_.swap(arg.deltaUff_);
		k_.swap(arg.k_);
	}

	void setZero(){
		uff_.setZero(); deltaUff_.setZero(); k_.setZero();
	}

	TimeArray time_;
	DiscreteArray<ct::core::ControlVector<CONTROL_DIM, SCALAR>> uff_;
	DiscreteArray<ct::core::ControlVector<CONTROL_DIM, SCALAR>> deltaUff_;
	DiscreteArray<Eigen::Matrix<SCALAR, CONTROL_DIM, STATE_DIM>> k_;
};




template <int STATE_DIM, int DIM1, int DIM2, class SCALAR = double>
class GeneralLinearFunction {

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	void swap(GeneralLinearFunction& arg){
		uff_.swap(arg.uff_);
		deltaUff_.swap(arg.deltaUff_);
		k_.swap(arg.k_);
	}

	void setZero(){
		uff_.setZero(); deltaUff_.setZero(); k_.setZero();
	}

	TimeArray time_;
	DiscreteArray<Eigen::Matrix<SCALAR, DIM1, DIM2>> uff_;
	DiscreteArray<Eigen::Matrix<SCALAR, DIM1, DIM2>> deltaUff_;
	DiscreteArray<Eigen::Matrix<SCALAR, DIM1, STATE_DIM>> k_;
};

} // core
} // ct

#endif /* CT_CORE_LINEARFUNCTIONS_H_ */
