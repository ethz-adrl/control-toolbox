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


#include <ct/core/core.h>
#include <ct/rbd/rbd.h>

#include <cmath>
#include <memory>

#include <ct/models/HyA/HyA.h>
#include <ct/rbd/systems/FixBaseFDSystem.h>

#include <ct/models/CodegenOutputDirs.h>

const size_t state_dim = ct::rbd::FixBaseFDSystem<ct::rbd::HyA::Dynamics>::STATE_DIM;
const size_t control_dim = ct::rbd::FixBaseFDSystem<ct::rbd::HyA::Dynamics>::CONTROL_DIM;
const size_t njoints = ct::rbd::HyA::Dynamics::NJOINTS;

typedef ct::core::ADCodegenLinearizer<state_dim, control_dim>::SCALAR Scalar;
typedef ct::rbd::FixBaseFDSystem<ct::rbd::HyA::tpl::Dynamics<Scalar>> HyANonLinearSystem;
typedef ct::core::DerivativesCppad<state_dim, control_dim> JacCG;

template<typename SCALAR>
using control_vector_t 	= typename ct::rbd::HyA::tpl::Dynamics<SCALAR>::control_vector_t;

template<typename SCALAR>
using ExtLinkForces_t		= typename ct::rbd::HyA::tpl::Dynamics<SCALAR>::ExtLinkForces_t;

// Computes the torque needed to compensate gravity
template <typename SCALAR>
Eigen::Matrix<SCALAR, control_dim, 1> hyaInverseDynamics(const Eigen::Matrix<SCALAR, state_dim, 1>& x)
{
 	ct::rbd::HyA::tpl::Dynamics<SCALAR> hyaDynamics;
	ct::rbd::tpl::JointState<njoints, SCALAR> hyaState(x);
	Eigen::Matrix<SCALAR, njoints, 1> qddTmp = Eigen::Matrix<SCALAR, njoints, 1>::Zero();
	ct::rbd::tpl::JointAcceleration<njoints, SCALAR> qdd(qddTmp); //zero
	ExtLinkForces_t<SCALAR> fext(Eigen::Matrix<SCALAR, njoints, 1>::Zero()); //zero
	control_vector_t<SCALAR> y;
 	hyaDynamics.FixBaseID(hyaState, qdd, fext, y);
	return y;
}

int main(int argc, char **argv){
	std::shared_ptr<HyANonLinearSystem> hya(new HyANonLinearSystem());
	ct::core::ADCodegenLinearizer<state_dim, control_dim> adLinearizer(hya);
	typename JacCG::FUN_TYPE_CG f = hyaInverseDynamics<CppAD::AD<CppAD::cg::CG<double> > >;
	JacCG jacCG(f);

	try 
	{
		std::cout << "Generating Jacobian of Inverse Dynamics wrt state using reverse mode... " << std::endl;
		jacCG.generateJacobianSource("HyAInverseDynJacReverse", ct::models::HYA_CODEGEN_OUTPUT_DIR, ct::core::CODEGEN_TEMPLATE_DIR, "models", "HyA", JacCG::Sparsity::Ones(), true);

        std::cout << "Generating Hessian of Inverse Dynamics wrt state using reverse mode... " << std::endl;
        jacCG.generateHessianSource("HyAInverseDynHessian", ct::models::HYA_CODEGEN_OUTPUT_DIR, ct::core::CODEGEN_TEMPLATE_DIR, "models", "HyA", JacCG::HessianSparsity::Ones(), true);

		std:: cout << "generating using forward mode" << std::endl;
		adLinearizer.generateCode("HyALinearizedForward", ct::models::HYA_CODEGEN_OUTPUT_DIR, ct::core::CODEGEN_TEMPLATE_DIR, "models", "HyA", false);

        std::cout << "generating using reverse mode" << std::endl;
        adLinearizer.generateCode("HyALinearizedReverse", ct::models::HYA_CODEGEN_OUTPUT_DIR, ct::core::CODEGEN_TEMPLATE_DIR, "models", "HyA", true);

		std::cout << "Generating Jacobian of Inverse Dynamics wrt state using forward mode... " << std::endl;
		jacCG.generateJacobianSource("HyAInverseDynJacForward", ct::models::HYA_CODEGEN_OUTPUT_DIR, ct::core::CODEGEN_TEMPLATE_DIR, "models", "HyA", JacCG::Sparsity::Ones(), false);

		std::cout<< "... done!" << std::endl;


	} catch (const std::runtime_error& e)
	{
		std::cout << "code generation failed: "<<e.what()<<std::endl;
	}

  	return 0;
}
