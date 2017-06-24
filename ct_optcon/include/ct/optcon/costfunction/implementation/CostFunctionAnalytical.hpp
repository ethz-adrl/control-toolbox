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


// add terms
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::addIntermediateTerm (std::shared_ptr< TermBase<STATE_DIM, CONTROL_DIM, SCALAR> > term, bool verbose)
{ 
	this->intermediateCostAnalytical_.push_back(term);
	if(verbose){
		std::string name;
	    term->getName(name);
	    std::cout<<"Trying to add term as intermediate"<<std::endl;
	}

	return this->intermediateCostAnalytical_.size()-1;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::addFinalTerm (std::shared_ptr< TermBase<STATE_DIM, CONTROL_DIM, SCALAR> > term,  bool verbose)
{ 
	this->finalCostAnalytical_.push_back(term);
	if(verbose){
		std::string name;
	    term->getName(name);
	    std::cout<<"Trying to add term as final"<<std::endl;
	}

	return this->finalCostAnalytical_.size()-1;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::loadFromConfigFile(const std::string& filename, bool verbose){
	this->intermediateCostAnalytical_.clear();
	this->finalCostAnalytical_.clear();

	boost::property_tree::ptree pt;
	boost::property_tree::read_info(filename, pt);
	int i=0;
	std::string currentTerm;
	do 
	{
		std::cout << "=============================================" << std::endl; //indicating new term
		currentTerm = "term"+std::to_string(i);
		std::string termKind = pt.get<std::string>(currentTerm + ".kind");
		boost::algorithm::to_lower(termKind);
		int currentTermType = pt.get<int>(currentTerm + ".type");
		std::string termName;
		try{
			termName = pt.get<std::string>(currentTerm + ".name");
			if(verbose)
				std::cout << "Trying to add "+termName+" as term" << std::endl;
		}
		catch(boost::property_tree::ptree_bad_path err) {
			termName = "Unnamed";
			if(verbose){
				std::cout<<"Name field for " +currentTerm+ " does not exist"<<std::endl;
			}
		}

		std::shared_ptr< TermBase<STATE_DIM, CONTROL_DIM, SCALAR> > term;

		CT_LOADABLE_TERMS_ANALYTICAL(SCALAR);

		if(!term){
			throw std::runtime_error("Term type \""+ termKind+ "\" not supported");
		} else
		{
			addTerm(filename,currentTerm,currentTermType,term,this,verbose);
		}
		currentTerm = "term"+std::to_string(++i);
	} while(pt.find(currentTerm)!= pt.not_found());
}

// evaluate
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
SCALAR CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::evaluateIntermediate()
{
	SCALAR y = 0.;
		
	for(auto it : this->intermediateCostAnalytical_)
	{
		if (!it->isActiveAtTime(this->t_)) { continue; }
		y += it->computeActivation(this->t_) * it->eval(this->x_, this->u_, this->t_);
	}
	
	return y;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
SCALAR CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::evaluateTerminal()
{
	SCALAR y = 0.;

	for(auto it : this->finalCostAnalytical_)
		y += it->evaluate(this->x_, this->u_, this->t_);

	return y;
}

// get state derivatives
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::state_vector_t CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::stateDerivativeIntermediate()
{
	state_vector_t derivative;
	derivative.setZero();

	for(auto it : this->intermediateCostAnalytical_)
	{
		if (!it->isActiveAtTime(this->t_)) { continue; }
		derivative += it->computeActivation(this->t_) * it->stateDerivative(this->x_, this->u_, this->t_);
	}

	return derivative;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::state_vector_t CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::stateDerivativeTerminal()
{
	state_vector_t derivative;
	derivative.setZero();

	for(auto it : this->finalCostAnalytical_)
		derivative += it->stateDerivative(this->x_, this->u_, this->t_);

	return derivative;
}

// get state second derivatives
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::state_matrix_t CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::stateSecondDerivativeIntermediate()
{
	state_matrix_t derivative;
	derivative.setZero(); 
	
	for(auto it : this->intermediateCostAnalytical_)
	{
		if (!it->isActiveAtTime(this->t_)) { continue; }
		derivative += it->computeActivation(this->t_) * it->stateSecondDerivative(this->x_, this->u_, this->t_);
	}

	return derivative;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::state_matrix_t CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::stateSecondDerivativeTerminal()
{
	state_matrix_t derivative;
	derivative.setZero();

	for(auto it : this->finalCostAnalytical_)
		derivative += it->stateSecondDerivative(this->x_, this->u_, this->t_);

	return derivative;
}

// get control derivatives
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::control_vector_t CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::controlDerivativeIntermediate()
{
	control_vector_t derivative;
	derivative.setZero();

	for(auto it : this->intermediateCostAnalytical_)
	{
		if (!it->isActiveAtTime(this->t_)) { continue; }
		derivative += it->computeActivation(this->t_) * it->controlDerivative(this->x_, this->u_, this->t_);
	}

	return derivative;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::control_vector_t CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::controlDerivativeTerminal()
{
	control_vector_t derivative;
	derivative.setZero();

	for(auto it : this->finalCostAnalytical_)
		derivative += it->controlDerivative(this->x_, this->u_, this->t_);

	return derivative;
}

// get control second derivatives
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::control_matrix_t CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::controlSecondDerivativeIntermediate()
{
	control_matrix_t derivative;
	derivative.setZero();
	
	for(auto it : this->intermediateCostAnalytical_)
	{
		if (!it->isActiveAtTime(this->t_)) { continue; }
		derivative += it->computeActivation(this->t_) * it->controlSecondDerivative(this->x_, this->u_, this->t_);
	}

	return derivative;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::control_matrix_t CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::controlSecondDerivativeTerminal()
{
	control_matrix_t derivative;
	derivative.setZero();
	
	for(auto it : this->finalCostAnalytical_)
		derivative += it->controlSecondDerivative(this->x_, this->u_, this->t_);

	return derivative;
}

// get state-control derivatives
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::control_state_matrix_t CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::stateControlDerivativeIntermediate()
{
	control_state_matrix_t derivative;
	derivative.setZero(); 
	
	for(auto it : this->intermediateCostAnalytical_)
	{
		if (!it->isActiveAtTime(this->t_)) { continue; }
		derivative += it->computeActivation(this->t_) * it->stateControlDerivative(this->x_, this->u_, this->t_);
	}

	return derivative;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::control_state_matrix_t CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::stateControlDerivativeTerminal()
{
	control_state_matrix_t derivative;
	derivative.setZero();
	
	for(auto it : this->finalCostAnalytical_)
		derivative += it->stateControlDerivative(this->x_, this->u_, this->t_);

	return derivative;
}
