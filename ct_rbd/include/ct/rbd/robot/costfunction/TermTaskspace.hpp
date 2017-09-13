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


#ifndef CT_RBD_COSTFUNCTION_TERM_TASKSPACE_HPP_
#define CT_RBD_COSTFUNCTION_TERM_TASKSPACE_HPP_

#include <ct/core/types/StateVector.h>
#include <ct/core/types/ControlVector.h>

#include <ct/optcon/costfunction/term/TermBase.hpp>
#include <ct/optcon/costfunction/term/utilities/TermTypedefs.hpp>
#include <ct/optcon/costfunction/utility/utilities.hpp>

#include <iit/rbd/traits/TraitSelector.h>
#include <ct/rbd/robot/Kinematics.h>
#include <ct/rbd/state/RBDState.h>


namespace ct{
namespace rbd{


/*!
 * \brief A costfunction term that defines a cost in task space
 *
 * This cost function adds a quadratic penalty on the position offset of an endeffector to a desired position
 *  @todo add velocity to term
 *
 * \tparam KINEMATICS kinematics of the system
 * \tparam FB true if system is a floating base robot
 * \tparam STATE_DIM state dimensionality of the system
 * \tparam CONTROL_DIM control dimensionality of the system
 */
template<class KINEMATICS, bool FB, size_t STATE_DIM, size_t CONTROL_DIM>
class TermTaskspace : public optcon::TermBase <STATE_DIM, CONTROL_DIM, CppAD::AD<double>, double>
{
public:
	typedef CppAD::AD<double> SCALAR;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	CT_OPTCON_DEFINE_TERM_TYPES

	TermTaskspace(
			size_t eeInd,
			const Eigen::Matrix<SCALAR, 3, 3>& Q,
			const core::StateVector<3, SCALAR>& x_des = core::StateVector<3, SCALAR>::Zero(),
			const std::string& name = "TermTaskSpace")
	:
		optcon::TermBase<STATE_DIM, CONTROL_DIM, CppAD::AD<double>, double>(name),
		eeInd_(eeInd),
		QTaskSpace_(Q),
		x_ref_(x_des)
 	{
 		currState_.setZero();
 		// Checks whether STATE_DIM has the appropriate size.
 		//  2 * (FB * 6 + KINEMATICS::NJOINTS)) represents a floating base system with euler angles
 		//  2 * (FB * 6 + KINEMATICS::NJOINTS) + 1 representing a floating base system with quaternion angles
 		static_assert((STATE_DIM == 2 * (FB * 6 + KINEMATICS::NJOINTS)) || (STATE_DIM == 2 * (FB * 6 + KINEMATICS::NJOINTS) + 1), "STATE_DIM does not have appropriate size.");
 	}

	TermTaskspace() = delete;

	virtual ~TermTaskspace(){}

	TermTaskspace(std::string& configFile, const std::string& termName, bool verbose = false)
	{
		loadConfigFile(configFile, termName, verbose);
	}

	virtual SCALAR evaluate(const Eigen::Matrix<SCALAR, STATE_DIM, 1> &x, const Eigen::Matrix<SCALAR, CONTROL_DIM, 1> &u, const SCALAR& t)
	{	
		setStateFromVector<FB>(x);
	    Eigen::Matrix<SCALAR, 3, 1> xDiff = kinematics_.getEEPositionInWorld(eeInd_, currState_.basePose(), currState_.jointPositions()).toImplementation() - x_ref_.template cast<SCALAR>();
	    return (xDiff.transpose() * QTaskSpace_.template cast<SCALAR>() * xDiff)(0,0);
	}

	TermTaskspace<KINEMATICS, FB, STATE_DIM, CONTROL_DIM>* clone () const override {
		return new TermTaskspace(*this);
	}

	void loadConfigFile(const std::string& filename, const std::string& termName, bool verbose = false) override
	{
		ct::optcon::loadScalarCF(filename,"eeId", eeInd_,termName);
		ct::optcon::loadMatrixCF(filename,"Q", QTaskSpace_,termName);
		ct::optcon::loadMatrixCF(filename,"x_des", x_ref_,termName);

		if (verbose){
		   std::cout<<"Read eeId as eeId = \n"<<eeInd_<<std::endl;
		   std::cout<<"Read Q as Q = \n"<<QTaskSpace_<<std::endl;
		   std::cout<<"Read x_des as x_des = \n"<<x_ref_.transpose() << std::endl;
		}
	}


private:
	size_t eeInd_;
	tpl::RBDState<KINEMATICS::NJOINTS, SCALAR> currState_;
	KINEMATICS kinematics_;

	Eigen::Matrix<SCALAR, 3, 3> QTaskSpace_;
	ct::core::StateVector<3, SCALAR> x_ref_;

	template <bool T>
	void setStateFromVector(const Eigen::Matrix<SCALAR, STATE_DIM, 1> &x, typename std::enable_if<T, bool>::type = true)
	{
		currState_.fromStateVectorRaw(x);
	}

	template <bool T>
	void setStateFromVector(const Eigen::Matrix<SCALAR, STATE_DIM, 1> &x, typename std::enable_if<!T, bool>::type = true)
	{
		currState_.joints() = x;
	}

};


} // namespace rbd
} // namespace ct




#endif // CT_RBD_COSTFUNCTION_TERM_TASKSPACE_HPP_
