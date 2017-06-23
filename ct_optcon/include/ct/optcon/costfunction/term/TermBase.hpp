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


#ifndef CT_OPTCON_TERMBASE_HPP_
#define CT_OPTCON_TERMBASE_HPP_

#include <ct/optcon/costfunction/term/timeActivations/TimeActivationBase.hpp>
#include <boost/algorithm/string.hpp>   

#include "timeActivations/TimeActivations.h"
#include "utilities/TermTypedefs.hpp"

namespace ct {
namespace optcon {

/*!
 * \ingroup CostFunction
 *
 * \brief An interface for a term, supporting both analytical and auto-diff terms
 *
 * Derive from this term to implement your own term. You only have to implement
 * evaluateIntermediate() if you want to use auto-diff. Otherwise, you have to implement the
 * derivatives as well.
 *
 * An example for an implementation of a custom term is given in \ref EEDistanceTerm.h
 **/
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR, typename TIME_SCALAR = SCALAR>
class TermBase {
private:
	std::string name_;
	std::shared_ptr<tpl::TimeActivationBase<TIME_SCALAR>> c_i_;

public:
	CT_OPTCON_DEFINE_TERM_TYPES

	/**
	 * \brief Default constructor
	 * @param name Name of the term
	 */
	TermBase(std::string name = "Unnamed") 
	: 
	name_(name),
	c_i_(std::shared_ptr<tpl::TimeActivationBase<TIME_SCALAR>> (new tpl::TimeActivationBase<TIME_SCALAR>()))
	{}

	/**
	 * \brief Copy Cunstructor
	 * @param arg The term to copy
	 */
	TermBase(const TermBase& arg): 
	name_(arg.name_),
	c_i_(arg.c_i_)
	{}

	/**
	 * \brief Deep-copy term
	 * @return
	 */
	virtual TermBase<STATE_DIM, CONTROL_DIM, SCALAR, TIME_SCALAR>* clone () const = 0;

	/**
	 * \brief Destructor
	 */
	virtual ~TermBase() {}
	
	/**
	 * \brief Evaluate term
	 * This function should evaluate the term.
	 * @param x State
	 * @param u Control
	 * @param t time
	 * @return
	 */
	virtual SCALAR evaluate(const Eigen::Matrix<SCALAR, STATE_DIM, 1> &x, const Eigen::Matrix<SCALAR, CONTROL_DIM, 1> &u, const SCALAR& t) = 0;

	// Todo: Make sure this gets only called by the analytical costfunction!!
	SCALAR eval(const Eigen::Matrix<SCALAR, STATE_DIM, 1> &x, const Eigen::Matrix<SCALAR, CONTROL_DIM, 1> &u, const SCALAR& t)
	{
		return computeActivation(t) * evaluate(x, u , t);
	}
	
	/**
	 * \brief Returns if term is non-zero at a specific time
	 * By default, all terms are evaluated at all times. However, if a term is not active at a certain time, you can overload this
	 * function to spare evaluations of the term and its derivatives
	 * @param t time
	 * @return true if term is active at t
	 */
	virtual bool isActiveAtTime(TIME_SCALAR t)
	{
		return c_i_->isActiveAtTime(t);
	}

	TIME_SCALAR computeActivation(TIME_SCALAR t)
	{
 		return c_i_->computeActivation(t); 		 
	}

	virtual core::StateVector<STATE_DIM, SCALAR> stateDerivative(const core::StateVector<STATE_DIM, SCALAR> &x, const core::ControlVector<CONTROL_DIM, SCALAR> &u, const SCALAR& t) { throw std::runtime_error("This cost function element is not implemented for the given term. Please use either auto-diff cost function or implement the analytical derivatives manually."); }

	virtual state_matrix_t stateSecondDerivative(const core::StateVector<STATE_DIM, SCALAR> &x, const core::ControlVector<CONTROL_DIM, SCALAR> &u, const SCALAR& t) { throw std::runtime_error("This cost function element is not implemented for the given term. Please use either auto-diff cost function or implement the analytical derivatives manually."); }
	
	virtual core::ControlVector<CONTROL_DIM, SCALAR> controlDerivative(const core::StateVector<STATE_DIM, SCALAR> &x, const core::ControlVector<CONTROL_DIM, SCALAR> &u, const SCALAR& t) { throw std::runtime_error("This cost function element is not implemented for the given term. Please use either auto-diff cost function or implement the analytical derivatives manually."); }
	
	virtual control_matrix_t controlSecondDerivative(const core::StateVector<STATE_DIM, SCALAR> &x, const core::ControlVector<CONTROL_DIM, SCALAR> &u, const SCALAR& t) { throw std::runtime_error("This cost function element is not implemented for the given term. Please use either auto-diff cost function or implement the analytical derivatives manually."); }

	virtual control_state_matrix_t stateControlDerivative(const core::StateVector<STATE_DIM, SCALAR> &x, const core::ControlVector<CONTROL_DIM, SCALAR> &u, const SCALAR& t) { throw std::runtime_error("This cost function element is not implemented for the given term. Please use either auto-diff cost function or implement the analytical derivatives manually."); }

	virtual void loadConfigFile(const std::string& filename, const std::string& termName, bool verbose = false) { throw std::runtime_error("This cost function element is not implemented for the given term. Please use either auto-diff cost function or implement the analytical derivatives manually."); }  // a pure virtual function for daa loading

	void setTimeActivation(std::shared_ptr<tpl::TimeActivationBase<TIME_SCALAR>> c_i, bool verbose = false)
	{
		c_i_ = c_i;
		if(verbose)
			c_i_->printInfo();
	}

	// this needs some testing
	void loadTimeActivation(const std::string& filename, const std::string& termName, bool verbose = false) {
		boost::property_tree::ptree pt;
		boost::property_tree::read_info(filename, pt);

		try{
			std::string activationKind = pt.get<std::string>(termName + ".time_activation" + ".kind");
			boost::algorithm::to_lower(activationKind);
			std::shared_ptr<tpl::TimeActivationBase<TIME_SCALAR>> c_i;
			CT_LOADABLE_TIME_ACTIVATIONS(TIME_SCALAR);
			c_i->loadConfigFile(filename, termName + ".time_activation", verbose);
			if(!c_i){
				throw std::runtime_error("Activation type \""+ activationKind+ "\" not supported");
			}
			else{
				c_i_ = c_i;
				if(verbose)
					c_i_->printInfo();
			}
		}
		catch(std::exception& e)
		{
			return;
		}
	}


		// throw std::runtime_error("This cost function element is not implemented for the given term. Please
		//  use either auto-diff cost function or implement the analytical derivatives manually."); }  // a pure virtual function for daa loading

	/**
	 * \brief Returns the name of the term
	 * @param termName name of the term
	 */
	void getName(std::string& termName) const {termName=name_;}

	/**
	 * \brief Sets the name of the term
	 * @param termName
	 */
	void setName(const std::string termName){name_=termName;}

	virtual void updateReferenceState (const Eigen::Matrix<SCALAR, STATE_DIM, 1>& newRefState){}
	virtual Eigen::Matrix<SCALAR, STATE_DIM, 1> getReferenceState() const {throw std::runtime_error("getReferenceState is not implemented for the current term!");}

protected:
};

} // namespace optcon
} // namespace ct

#endif // TERMBASE_HPP_
