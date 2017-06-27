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

#ifndef CT_OPTCON_COSTFUNCTION_QUADRATIC_HPP_
#define CT_OPTCON_COSTFUNCTION_QUADRATIC_HPP_

#include "CostFunction.hpp"
#include "term/TermBase.hpp"

namespace ct {
namespace optcon {


/**
 * \ingroup CostFunction
 *
 * \brief Describes a cost function with a quadratic approximation, i.e. one that
 * can compute first and second order derivatives with respect to state and
 * control input. **This does NOT mean it has to be a purely quadratic cost
 * function**. If you are looking for a purely quadratic cost function, check
 * CostFunctionQuadraticSimple.
 *
 * A cost function is assumed to be a sum of intermediate and final terms, i.e.
 * \f$ J(x,u,t) = \sum_{n=0}^{N_i} T_{i,n}(x,u,t) + \sum_{n=0}^{N_f} T_{i,f}(x,u,t) \f$
 * These terms can have arbitrary form.
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class CostFunctionQuadratic : public CostFunction<STATE_DIM, CONTROL_DIM, SCALAR> {

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM> state_matrix_t;
	typedef Eigen::Matrix<SCALAR, CONTROL_DIM, CONTROL_DIM> control_matrix_t;
	typedef Eigen::Matrix<SCALAR, CONTROL_DIM, STATE_DIM> control_state_matrix_t;

	typedef core::StateVector<STATE_DIM, SCALAR> state_vector_t;
	typedef core::ControlVector<CONTROL_DIM, SCALAR> control_vector_t;

	typedef CostFunction<STATE_DIM, CONTROL_DIM, SCALAR> BASE;

	/**
	 * Constructor
	 */
	CostFunctionQuadratic()
	{
		eps_ = sqrt(Eigen::NumTraits<SCALAR>::epsilon() );
	}

	/**
	 * Constructor given a setpoint
	 * @param x state vector
	 * @param u control vector
	 * @param t time
	 */
	CostFunctionQuadratic(const state_vector_t &x, const control_vector_t &u, const SCALAR& t):
		CostFunction<STATE_DIM, CONTROL_DIM, SCALAR>(x, u, t)
	{
		eps_ = sqrt(Eigen::NumTraits<SCALAR>::epsilon() );
	}

	/**
	 * Copy constructor
	 * @param arg other cost function
	 */
	CostFunctionQuadratic(const CostFunctionQuadratic& arg):
		CostFunction<STATE_DIM, CONTROL_DIM, SCALAR>(arg),
		eps_(arg.eps_),
		doubleSidedDerivative_(arg.doubleSidedDerivative_)
	{
		intermediateCostAnalytical_.resize(arg.intermediateCostAnalytical_.size());
		finalCostAnalytical_.resize(arg.finalCostAnalytical_.size());

		for(size_t i = 0; i<arg.intermediateCostAnalytical_.size(); i++)
		{
			intermediateCostAnalytical_[i] = std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR>> (arg.intermediateCostAnalytical_[i]->clone());
		}

		for(size_t i = 0; i<arg.finalCostAnalytical_.size(); i++)
		{
			finalCostAnalytical_[i] = std::shared_ptr< TermBase<STATE_DIM, CONTROL_DIM, SCALAR> > (arg.finalCostAnalytical_[i]->clone());
		}
	}

	/**
	 * Clones the cost function.
	 * @return
	 */
	virtual CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>* clone () const = 0;

	/**
	 * Destructor
	 */
	virtual ~CostFunctionQuadratic() {}

	/**
	 * \brief Adds an intermediate term
	 * @param term intermediate term
	 * @param verbose verbosity flag which enables printout
	 * @return
	 */
	virtual size_t addIntermediateTerm (std::shared_ptr< TermBase<STATE_DIM, CONTROL_DIM, SCALAR> > term, bool verbose = false) { throw std::runtime_error("not implemented"); };

	/**
	 * \brief Adds a final term
	 * @param term final term
	 * @param verbose verbosity flag which enables printout
	 * @return
	 */
	virtual size_t addFinalTerm (std::shared_ptr< TermBase<STATE_DIM, CONTROL_DIM, SCALAR> > term, bool verbose = false) { throw std::runtime_error("not implemented"); };

	/**
	 * \brief Loads cost function from config file
	 * @param filename config file location
	 * @param verbose verbosity flag which enables printout
	 */
	virtual void loadFromConfigFile(const std::string& filename, bool verbose = false)  { throw std::runtime_error("not implemented"); };

	/**
	 * \brief Computes intermediate-cost first-order derivative with respect to state
	 * @return derivative vector (Jacobian)
	 */
	virtual state_vector_t stateDerivativeIntermediate() = 0;

	/**
	 * Computes terminal-cost first-order derivative with respect to state
	 * @return derivative vector (Jacobian)
	 */
	virtual state_vector_t stateDerivativeTerminal() = 0;

	/**
	 * \brief Computes intermediate-cost second-order derivative with respect to state
	 * @return derivative matrix (Jacobian)
	 */
	virtual state_matrix_t stateSecondDerivativeIntermediate() = 0;

	/**
	 * \brief Computes final-cost second-order derivative with respect to state
	 * @return derivative matrix (Jacobian)
	 */
	virtual state_matrix_t stateSecondDerivativeTerminal() = 0;

	/**
	 * \brief Computes intermediate-cost first-order derivative with respect to control
	 * @return derivative vector (Jacobian)
	 */
	virtual control_vector_t controlDerivativeIntermediate() = 0;

	/**
	 * \brief Computes terminal-cost first-order derivative with respect to control
	 *
	 * Not available for all cost functions. Throws an exception if not available.
	 * @return derivative vector (Jacobian)
	 */
	virtual control_vector_t controlDerivativeTerminal() { throw std::runtime_error("controlDerivativeTerminal() not implemented in CostFunctionQuadratic"); };

	/**
	 * \brief Computes intermediate-cost second-order derivative with respect to input
	 * @return derivative matrix (Jacobian)
	 */
	virtual control_matrix_t controlSecondDerivativeIntermediate() = 0;

	/**
	 * \brief Computes final-cost second-order derivative with respect to input
	 *
	 * Not available for all cost functions. Throws an exception if not available.
	 * @return derivative matrix (Jacobian)
	 */
	virtual control_matrix_t controlSecondDerivativeTerminal() { throw std::runtime_error("controlSecondDerivativeTerminal() not implemented"); };

	/**
	 * \brief Computes intermediate-cost derivative with respect to state and control
	 * @return derivative matrix (Jacobian)
	 */
	virtual control_state_matrix_t stateControlDerivativeIntermediate() = 0;

	/**
	 * \brief Computes final-cost derivative with respect to state and control
	 * @return derivative matrix (Jacobian)
	 */
	virtual control_state_matrix_t stateControlDerivativeTerminal() { throw std::runtime_error("stateControlDerivativeTerminal() not implemented"); };

	virtual void updateReferenceState(const state_vector_t& x_ref)
	{
		for(auto costIntermediate : intermediateCostAnalytical_)
			costIntermediate->updateReferenceState(x_ref);
	}

	virtual void updateFinalState(const state_vector_t& x_final)
	{
		for(auto costFinal : finalCostAnalytical_)
			costFinal->updateReferenceState(x_final);
	}


	bool stateDerivativeIntermediateTest()
	{
		state_vector_t derivative = stateDerivativeIntermediate();
		state_vector_t derivativeNd = stateDerivativeIntermediateNumDiff();
		std::cout << "norm error between derivative/numdiff state : "<<std::endl<<(derivative-derivativeNd).norm()<<std::endl;

		return (derivative.isApprox(derivativeNd, 1e-6));
	}

	bool controlDerivativeIntermediateTest()
	{
		control_vector_t derivative = controlDerivativeIntermediate();
		control_vector_t derivativeNd = controlDerivativeIntermediateNumDiff();
		std::cout << "norm error between derivative/numdiff control : "<<std::endl<<(derivative-derivativeNd).norm()<<std::endl;

		return (derivative.isApprox(derivativeNd, 1e-6));
	}

	std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR>> getIntermediateTermById(size_t id)
	{
		return intermediateCostAnalytical_[id];
	}

	std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR>> getFinalTermById(size_t id)
	{
		return finalCostAnalytical_[id];
	}


protected:

	state_vector_t stateDerivativeIntermediateNumDiff();
	control_vector_t controlDerivativeIntermediateNumDiff();

	SCALAR eps_;
	bool doubleSidedDerivative_ = true;

	std::vector < std::shared_ptr< TermBase<STATE_DIM, CONTROL_DIM, SCALAR> > > intermediateCostAnalytical_; /** list of intermediate cost terms for which analytic derivatives are available */
	std::vector < std::shared_ptr< TermBase<STATE_DIM, CONTROL_DIM, SCALAR> > > finalCostAnalytical_; /** list of final cost terms for which analytic derivatives are available */
};

#include "implementation/CostFunctionQuadratic-impl.hpp"


} // namespace optcon
} // namespace ct

#endif // COSTFUNCTION_HPP_
