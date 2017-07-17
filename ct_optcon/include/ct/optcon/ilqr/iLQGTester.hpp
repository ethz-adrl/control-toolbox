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

#ifndef INCLUDE_CT_OPTCON_ILQG_ILQGTESTER_HPP_
#define INCLUDE_CT_OPTCON_ILQG_ILQGTESTER_HPP_

namespace ct{
namespace optcon{



/*!
 * \ingroup iLQG
 *
 * \brief Tester for iLQG
 * This class is just a helper class that checks the consistency of
 * dynamics, costs and linear systems passed to ilqg_. The reason
 * this class is separate is to keep the iLQGBase class clutter free.
 */
template <class iLQG>
class iLQGTester
{

public:
	static const size_t STATE_DIM = iLQG::STATE_D;
	static const size_t CONTROL_DIM = iLQG::CONTROL_D;


	iLQGTester(const iLQG& ilqg):
		ilqg_(ilqg)
	{

	}

	bool testAllConsistency(bool verbose)
	{
		return testSystemDynamicsConsistency(verbose) &&
				testLinearSystemsConsistency(verbose) &&
				testCostFunctionConsistency(verbose);
	}

	bool testSystemDynamicsConsistency(bool verbose)
	{
		size_t nTests = 10000;

		if (verbose)
			std::cout << "Running system dynamics consistency check with "<<nTests<<" test data points."<<std::endl;

		for (size_t i=0; i<nTests; i++)
		{
			ct::core::ControlVector<CONTROL_DIM> u;
			u.setRandom();

			ct::core::StateVector<STATE_DIM> x;
			x.setRandom();

			std::vector<ct::core::StateVector<STATE_DIM>> xdot(ilqg_.systems_.size());

			for (size_t threadId=0; threadId<ilqg_.systems_.size(); threadId++)
			{
				ilqg_.systems_[threadId]->computeControlledDynamics(x, 0.0, u, xdot[threadId]);
			}

			bool failed = false;

			for (size_t threadId=1; threadId<ilqg_.systems_.size(); threadId++)
			{
				if (xdot[threadId] != xdot[0])
				{
					if (verbose)
						std::cout << "Dynamics consistency test failed for dynamics 0 and " <<threadId <<std::endl;

					failed = true;
				}
			}
			if (failed)
				return false;
		}
		if (verbose)
			std::cout << "System dynamics consistency check succeeded."<<std::endl;

		return true;
	}

	bool testLinearSystemsConsistency(bool verbose)
	{
		size_t nTests = 10000;

		if (verbose)
			std::cout << "Running linear system consistency check with "<<nTests<<" test data points."<<std::endl;

		for (size_t i=0; i<nTests; i++)
		{
			ct::core::ControlVector<CONTROL_DIM> u;
			u.setRandom();

			ct::core::StateVector<STATE_DIM> x;
			x.setRandom();

			typename iLQG::StateMatrixArray A(ilqg_.linearSystems_.size());
			typename iLQG::StateControlMatrixArray B(ilqg_.linearSystems_.size());

			for (size_t threadId=0; threadId<ilqg_.linearSystems_.size(); threadId++)
			{
				A[threadId] = ilqg_.linearSystems_[threadId]->getDerivativeState(x, u, 0.0);
				B[threadId] = ilqg_.linearSystems_[threadId]->getDerivativeControl(x, u, 0.0);
			}

			bool failed = false;

			for (size_t threadId=1; threadId<ilqg_.linearSystems_.size(); threadId++)
			{
				if (A[threadId] != A[0] ||
					B[threadId] != B[0]
				)
				{
					if (verbose)
						std::cout << "Linear systems consistency test failed for dynamics 0 and " <<threadId <<std::endl;

					failed = true;
				}
			}
			if (failed)
				return false;
		}
		if (verbose)
			std::cout << "linear system consistency check succeeded."<<std::endl;

		return true;
	}


	bool testCostFunctionConsistency(bool verbose)
	{
		size_t nTests = 10000;

		if (verbose)
			std::cout << "Running cost function consistency check with "<<nTests<<" test data points."<<std::endl;

		for (size_t i=0; i<nTests; i++)
		{
			ct::core::ControlVector<CONTROL_DIM> u;
			u.setRandom();

			ct::core::StateVector<STATE_DIM> x;
			x.setRandom();

			typename iLQG::scalar_array_t q(ilqg_.costFunctions_.size());
			typename iLQG::StateVectorArray qv(ilqg_.costFunctions_.size());
			typename iLQG::StateMatrixArray Q(ilqg_.costFunctions_.size());
			typename iLQG::FeedbackArray P(ilqg_.costFunctions_.size());
			typename iLQG::ControlVectorArray rv(ilqg_.costFunctions_.size());
			typename iLQG::ControlMatrixTrajectory R(ilqg_.costFunctions_.size());

			for (size_t threadId=0; threadId<ilqg_.costFunctions_.size(); threadId++)
			{
				// feed current state and control to cost function
				ilqg_.costFunctions_[threadId]->setCurrentStateAndControl(x, u, 0.0);

				// derivative of cost with respect to state
				q[threadId] = ilqg_.costFunctions_[threadId]->evaluateIntermediate();

				qv[threadId] = ilqg_.costFunctions_[threadId]->stateDerivativeIntermediate();

				Q[threadId] = ilqg_.costFunctions_[threadId]->stateSecondDerivativeIntermediate();

				// derivative of cost with respect to control and state
				P[threadId] = ilqg_.costFunctions_[threadId]->stateControlDerivativeIntermediate();

				// derivative of cost with respect to control
				rv[threadId] = ilqg_.costFunctions_[threadId]->controlDerivativeIntermediate();

				R[threadId] = ilqg_.costFunctions_[threadId]->controlSecondDerivativeIntermediate();
			}

			bool failed = false;

			for (size_t threadId=1; threadId<ilqg_.costFunctions_.size(); threadId++)
			{
				if (q[threadId] != q[0] ||
					qv[threadId] != qv[0] ||
					Q[threadId] != Q[0] ||
					P[threadId] != P[0] ||
					rv[threadId] != rv[0] ||
					R[threadId] != R[0]
				)
				{
					if (verbose)
						std::cout << "cost function consistency test failed for cost function 0 and " <<threadId <<std::endl;

					failed = true;
				}
			}
			if (failed)
				return false;
		}
		if (verbose)
			std::cout << "cost function consistency check succeeded."<<std::endl;

		return true;
	}

private:
	const iLQG& ilqg_;

};


}
}


#endif /* INCLUDE_CT_OPTCON_ILQG_ILQGTESTER_HPP_ */
