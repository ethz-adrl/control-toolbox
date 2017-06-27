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

#ifndef CT_SCALAR_TRAJECTORY_H_
#define CT_SCALAR_TRAJECTORY_H_

#include "DiscreteTrajectoryBase.h"

namespace ct {
namespace core {


//! Specialized type of a discrete trajectory for scalar types
/*!
 * \tparam SCALAR scalar data type
 */
template<class SCALAR = double, class TIME_SCALAR = double>
class ScalarTrajectory : public DiscreteTrajectoryBase<SCALAR, Eigen::aligned_allocator<SCALAR>, TIME_SCALAR>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	//! default constructor
	ScalarTrajectory() {};

	//! resize constructor
	/*!
	 * initializes an array with a certain length and fills it with a default value
	 * @param n length of array
	 * @param value default value
	 */
	ScalarTrajectory(size_t n, const SCALAR& value=SCALAR())
		: DiscreteTrajectoryBase<SCALAR>(n,value)  {};

	//! copy constructor
	ScalarTrajectory(const ScalarTrajectory& other)
		: DiscreteTrajectoryBase<SCALAR>(other)  {};

	//! constructor from std::vector
	ScalarTrajectory(const std::vector<SCALAR>& arg):
		DiscreteTrajectoryBase<SCALAR>(){
		for(size_t i = 0; i<arg.size(); i++)
			this->push_back(arg[i]);
	}

	//! destructor
	virtual ~ScalarTrajectory(){}

	//! convert to an Eigen trajectory
	std::vector<Eigen::Matrix<SCALAR, 1, 1>, Eigen::aligned_allocator<Eigen::Matrix<SCALAR, 1, 1>>> toEigenTrajectory(){
		std::vector<Eigen::Matrix<SCALAR, 1, 1>, Eigen::aligned_allocator<Eigen::Matrix<SCALAR, 1, 1>>> eigenTraj;
		for(size_t i = 0; i<this->size(); i++){
			Eigen::Matrix<SCALAR, 1, 1> newElement;
			newElement(0,0) = (*this)[i];
			eigenTraj.push_back(newElement);
		}
		return eigenTraj;
	}

private:

};

}
}

#endif /* CT_SCALAR_TRAJECTORY_H_ */
