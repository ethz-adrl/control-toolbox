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

template <size_t STATE_DIM, size_t CONTROL_DIM>
CostFunctionAD<STATE_DIM, CONTROL_DIM>::CostFunctionAD() :
CostFunctionQuadratic<STATE_DIM, CONTROL_DIM> ()
{
	var_.resize(STATE_DIM + CONTROL_DIM +1);
	setCurrentStateAndControl(this->x_, this->u_, this->t_);
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
CostFunctionAD<STATE_DIM, CONTROL_DIM>::CostFunctionAD(const state_vector_t &x, const control_vector_t &u, const double& t) :
CostFunctionQuadratic<STATE_DIM, CONTROL_DIM> (x,u, t)
{
	var_.resize(STATE_DIM + CONTROL_DIM +1);
	setCurrentStateAndControl(x, u, this->t_);
}


template <size_t STATE_DIM, size_t CONTROL_DIM>
CostFunctionAD<STATE_DIM, CONTROL_DIM>::CostFunctionAD(const CostFunctionAD& arg):
CostFunctionQuadratic<STATE_DIM, CONTROL_DIM>(arg),
var_(arg.var_),
reverse1_derivative_intermediate_(arg.reverse1_derivative_intermediate_),
reverse1_derivative_final_(arg.reverse1_derivative_final_),
hessian_state_intermediate_(arg.hessian_state_intermediate_),
hessian_control_intermediate_(arg.hessian_control_intermediate_),
hessian_control_state_intermediate_(arg.hessian_control_state_intermediate_),
hessian_state_final_(arg.hessian_state_final_),
hessian_control_final_(arg.hessian_control_final_),
hessian_control_state_final_(arg.hessian_control_state_final_)
{
	intermediateCostAD_.resize(arg.intermediateCostAD_.size());
	finalCostAD_.resize(arg.finalCostAD_.size());

	// initialize with a vector of pointers to empty ADFun objects
	intermediateFunctionAD_ = std::vector<std::shared_ptr<CppAD::ADFun<double>>> (
			arg.intermediateFunctionAD_.size(), std::shared_ptr<CppAD::ADFun<double>> (new CppAD::ADFun<double> ()));

	// initialize with a vector of pointers to empty ADFun objects
	finalFunctionAD_ = std::vector<std::shared_ptr<CppAD::ADFun<double>>> (
			arg.finalFunctionAD_.size(), std::shared_ptr<CppAD::ADFun<double>> (new CppAD::ADFun<double> ()));


	for(size_t i = 0; i<arg.intermediateCostAD_.size(); i++)
	{
		intermediateCostAD_[i] = std::shared_ptr< TermBaseAD> (arg.intermediateCostAD_[i]->clone());
	}

	for(size_t i = 0; i<arg.finalCostAD_.size(); i++)
	{
		finalCostAD_[i] = std::shared_ptr< TermBaseAD> (arg.finalCostAD_[i]->clone());
	}

	for(size_t i = 0; i<arg.intermediateFunctionAD_.size(); i++)
	{
		// explicitly copy the elements, not the pointers!
		*(intermediateFunctionAD_[i]) = *(arg.intermediateFunctionAD_[i]);
	}

	for(size_t i = 0; i<arg.finalCostAD_.size(); i++)
	{
		// explicitly copy the elements, not the pointers!

		if(finalFunctionAD_[i] == nullptr)
			throw std::runtime_error("is nullptr");

		*(finalFunctionAD_[i]) = *(arg.finalFunctionAD_[i]);
	}
}


template <size_t STATE_DIM, size_t CONTROL_DIM>
void CostFunctionAD<STATE_DIM, CONTROL_DIM>::termChanged(size_t termId) {
	recordTerm(intermediateCostAD_[termId], *intermediateFunctionAD_[termId]);
}

// add terms
template <size_t STATE_DIM, size_t CONTROL_DIM>
size_t CostFunctionAD<STATE_DIM, CONTROL_DIM>::addIntermediateTerm (std::shared_ptr< TermBase<STATE_DIM, CONTROL_DIM, double> > term, bool verbose)
{ 
	this->intermediateCostAnalytical_.push_back(term);
	if(verbose){
		std::string name;
		term->getName(name);
		std::cout<<name+" added as intermediate term"<<std::endl;
	}

	return this->intermediateCostAnalytical_.size()-1;
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
size_t CostFunctionAD<STATE_DIM, CONTROL_DIM>::addIntermediateTerm (std::shared_ptr< TermBaseAD > term, bool verbose)
{ 
	intermediateCostAD_.push_back(term);
	intermediateFunctionAD_.push_back(std::shared_ptr<CppAD::ADFun<double>>(new CppAD::ADFun<double>));
	recordTerm(intermediateCostAD_.back(), *intermediateFunctionAD_.back());

	if(verbose){
		std::string name;
		term->getName(name);
		std::cout<<name+" added as intermediate AD term"<<std::endl;
	}

	return intermediateCostAD_.size()-1;
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
size_t CostFunctionAD<STATE_DIM, CONTROL_DIM>::addFinalTerm (std::shared_ptr< TermBase<STATE_DIM, CONTROL_DIM, double> > term, bool verbose)
{ 
	this->finalCostAnalytical_.push_back(term);
	if(verbose){
		std::string name;
		term->getName(name);
		std::cout<<name+"added as final term"<<std::endl;
	}

	return this->finalCostAnalytical_.size()-1;
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
size_t CostFunctionAD<STATE_DIM, CONTROL_DIM>::addFinalTerm (std::shared_ptr< TermBaseAD > term, bool verbose)
{ 
	finalCostAD_.push_back(term);

	finalFunctionAD_.push_back(std::shared_ptr<CppAD::ADFun<double>>(new CppAD::ADFun<double>));
	recordTerm(finalCostAD_.back(), *finalFunctionAD_.back());

	if(verbose){
		std::string name;
		term->getName(name);
		std::cout<<name+"added as final AD term"<<std::endl;
	}

	return finalCostAD_.size()-1;
}

// set state and control
template <size_t STATE_DIM, size_t CONTROL_DIM>
void CostFunctionAD<STATE_DIM, CONTROL_DIM>::setCurrentStateAndControl(
		const state_vector_t &x,
		const control_vector_t &u,
		const double& t){
	this->x_ = x;
	this->u_ = u;
	this->t_ = t;

	var_ << x, u, t;

	// reset some previous results
	reverse1_derivative_intermediate_.setZero();
	reverse1_derivative_final_.setZero();

	// perform calculations that are general - reverse terms for first derivatives
	reverseADTerms(intermediateCostAD_, intermediateFunctionAD_, reverse1_derivative_intermediate_);
	reverseADTerms(finalCostAD_, finalFunctionAD_, reverse1_derivative_final_);

	// perform calculations that are general - get second derivatives
	getHessians(intermediateCostAD_, intermediateFunctionAD_, this->intermediateCostAnalytical_,
			hessian_state_intermediate_, hessian_control_intermediate_, hessian_control_state_intermediate_);

	getHessians(finalCostAD_, finalFunctionAD_, this->finalCostAnalytical_,
			hessian_state_final_, hessian_control_final_, hessian_control_state_final_);
}


template <size_t STATE_DIM, size_t CONTROL_DIM>
void CostFunctionAD<STATE_DIM, CONTROL_DIM>::loadFromConfigFile(const std::string& filename, bool verbose){
	boost::property_tree::ptree pt;
	boost::property_tree::read_info(filename, pt);
	int i=0;
	std::string currentTerm;
	do 
	{
		currentTerm = "term"+std::to_string(i);
		std::string termKind = pt.get<std::string>(currentTerm + ".kind");
		boost::algorithm::to_lower(termKind);
		int currentTermType = pt.get<int>(currentTerm + ".type");
		std::string termName;
		try{
			termName = pt.get<std::string>(currentTerm + ".name");
		}
		catch(boost::property_tree::ptree_bad_path err) {
			termName = "Unnamed";
			if(verbose){
				std::cout<<"Name field for " +currentTerm+ " does not exist"<<std::endl;
			}
		}

		std::shared_ptr< TermBase<STATE_DIM, CONTROL_DIM, double> > term;
		std::shared_ptr< TermBaseAD > termAD;

		CT_LOADABLE_TERMS_ANALYTICAL(double);
		CT_LOADABLE_TERMS_AD;

		if(!term && !termAD){
			throw std::runtime_error("Term type \""+ termKind+ "\" not supported");
		} else
		{
			if (term)
				addTerm(filename,currentTerm,currentTermType,term,this,verbose);
			if (termAD)
				addTerm(filename,currentTerm,currentTermType,termAD,this,verbose);
		}
		currentTerm = "term"+std::to_string(++i);
	} while(pt.find(currentTerm)!= pt.not_found());
}


// evaluate
template <size_t STATE_DIM, size_t CONTROL_DIM>
double CostFunctionAD<STATE_DIM, CONTROL_DIM>::evaluateIntermediate()
{
	return evaluateCost(intermediateCostAD_, intermediateFunctionAD_, this->intermediateCostAnalytical_);
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
double CostFunctionAD<STATE_DIM, CONTROL_DIM>::evaluateTerminal()
{
	return evaluateCost(finalCostAD_, finalFunctionAD_, this->finalCostAnalytical_);
}


template <size_t STATE_DIM, size_t CONTROL_DIM>
double CostFunctionAD<STATE_DIM, CONTROL_DIM>::evaluateCost(
		std::vector<std::shared_ptr<TermBaseAD>>& termsAD,
		std::vector<std::shared_ptr<CppAD::ADFun<double>>>& functionAD,
		std::vector<std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, double>>>& termsAnalytical){

	double cost = 0.0;

	// analytical part of the cost
	for(auto it : termsAnalytical)
	{
		if (!it->isActiveAtTime(this->t_)) { continue; }
		cost += it->computeActivation(this->t_) * it->evaluate(this->x_, this->u_, this->t_);
	}

	// evaluate AD part of the cast via zero-order forward mode
	for(size_t termsId = 0; termsId < termsAD.size(); termsId++) {
		if (!termsAD[termsId]->isActiveAtTime(this->t_)) { continue; }
		CppAD::AD<double> Y = functionAD[termsId]->Forward(0, var_)(0,0);
		cost += termsAD[termsId]->computeActivation(this->t_) * CppAD::Value(Y);
		// cost += termsAD[termsId]->CppAD::Value(Y);
	}

	return cost;
}


template <size_t STATE_DIM, size_t CONTROL_DIM>
typename CostFunctionAD<STATE_DIM, CONTROL_DIM>::state_vector_t CostFunctionAD<STATE_DIM, CONTROL_DIM>::stateDerivativeIntermediate()
{
	Eigen::Matrix<double, STATE_DIM, 1> derivative;
	derivative.setZero();

	for(auto it : this->intermediateCostAnalytical_)
	{
		if (!it->isActiveAtTime(this->t_)) { continue; }
		derivative+= it->computeActivation(this->t_) * it->stateDerivative(this->x_, this->u_, this->t_);
	}

	// add AD part
	derivative += reverse1_derivative_intermediate_.segment(0, STATE_DIM);

	return derivative;
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
typename CostFunctionAD<STATE_DIM, CONTROL_DIM>::state_vector_t CostFunctionAD<STATE_DIM, CONTROL_DIM>::stateDerivativeTerminal()
{
	Eigen::Matrix<double, STATE_DIM, 1> derivative;
	derivative.setZero();

	for(auto it : this->finalCostAnalytical_)
		derivative+= it->stateDerivative(this->x_, this->u_, this->t_);

	// add AD part
	derivative += reverse1_derivative_final_.segment(0, STATE_DIM);

	return derivative;
}


template <size_t STATE_DIM, size_t CONTROL_DIM>
typename CostFunctionAD<STATE_DIM, CONTROL_DIM>::control_vector_t CostFunctionAD<STATE_DIM, CONTROL_DIM>::controlDerivativeIntermediate()
{
	Eigen::Matrix<double, CONTROL_DIM, 1> derivative;
	derivative.setZero();

	for(auto it : this->intermediateCostAnalytical_)
	{
		if (!it->isActiveAtTime(this->t_)) { continue; }
		derivative+= it->computeActivation(this->t_) * it->controlDerivative(this->x_, this->u_, this->t_);
	}

	// add AD part
	derivative += reverse1_derivative_intermediate_.segment(STATE_DIM, CONTROL_DIM);

	return derivative;
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
typename CostFunctionAD<STATE_DIM, CONTROL_DIM>::control_vector_t CostFunctionAD<STATE_DIM, CONTROL_DIM>::controlDerivativeTerminal()
{
	Eigen::Matrix<double, CONTROL_DIM, 1> derivative;
	derivative.setZero();

	for(auto it : this->finalCostAnalytical_)
		derivative+= it->controlDerivative(this->x_, this->u_, this->t_);

	// add AD part
	derivative += reverse1_derivative_final_.segment(STATE_DIM, CONTROL_DIM);

	return derivative;
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
void CostFunctionAD<STATE_DIM, CONTROL_DIM>::getHessians(
		std::vector<std::shared_ptr<TermBaseAD>>& termsAD,
		std::vector<std::shared_ptr<CppAD::ADFun<double>>>& functionAD,
		std::vector<std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, double>>>& termsAnalytical,
		Eigen::Matrix<double, STATE_DIM,   STATE_DIM>&   hessian_state_,
		Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM>& hessian_control_,
		Eigen::Matrix<double, CONTROL_DIM, STATE_DIM>&   hessian_control_state_){

	Eigen::Matrix<double, Eigen::Dynamic, 1> domain_input(STATE_DIM+CONTROL_DIM+1);

	domain_input << this->x_, this->u_, this->t_;

	Eigen::Matrix<double, Eigen::Dynamic, 1> w (1);
	w.setConstant(1.0);

	Eigen::VectorXd hessian ((STATE_DIM+CONTROL_DIM+1)*(STATE_DIM+CONTROL_DIM+1)); hessian.setZero();		// temporary data, for evaluation and transcripion only


	for (size_t termId=0; termId<termsAD.size(); termId++){
		if (!termsAD[termId]->isActiveAtTime(this->t_)) { continue; }
		hessian += termsAD[termId]->computeActivation(this->t_) * functionAD[termId]->Hessian(domain_input, w);
	}

	// extract pure state hessian, AD part
	for(size_t i = 0; i< STATE_DIM; i++)
		for(size_t j = 0; j< STATE_DIM; j++)
			hessian_state_(i,j) = hessian(i*(STATE_DIM+CONTROL_DIM+1)+j);

	// extract pure control hessian, AD part
	for(size_t i = STATE_DIM; i< STATE_DIM+CONTROL_DIM; i++)
		for(size_t j = STATE_DIM; j< STATE_DIM+CONTROL_DIM; j++)
			hessian_control_(i-STATE_DIM,j-STATE_DIM) = hessian(i*(STATE_DIM+CONTROL_DIM+1)+j);

	// extract mixed control-state hessian, AD part
	for(size_t i = 0; i< STATE_DIM; i++)
		for(size_t j = STATE_DIM; j< STATE_DIM+CONTROL_DIM; j++)
			hessian_control_state_(j-STATE_DIM,i) = hessian(i*(STATE_DIM+CONTROL_DIM+1)+j);


	// take into account analytical terms
	for(size_t termId = 0; termId<termsAnalytical.size(); termId++){

		// extract pure state hessian, AD part
		hessian_state_ += termsAnalytical[termId]->stateSecondDerivative(this->x_, this->u_, this->t_);

		// extract pure control hessian, AD part
		hessian_control_ += termsAnalytical[termId]->controlSecondDerivative(this->x_, this->u_, this->t_);

		// extract mixed control-state hessian, AD part
		hessian_control_state_ += termsAnalytical[termId]->stateControlDerivative(this->x_, this->u_, this->t_);
	}
}


// record functions
template <size_t STATE_DIM, size_t CONTROL_DIM>
void CostFunctionAD<STATE_DIM, CONTROL_DIM>::recordTerm(const std::shared_ptr< TermBaseAD > &costAD, CppAD::ADFun<double>& functionAD)
{
	Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, 1> VAR(STATE_DIM + CONTROL_DIM + 1); 	// +1 for the time
	Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, 1> Y(1);
	Y.setZero();

	CppAD::Independent(VAR);

	Eigen::Matrix<CppAD::AD<double>, STATE_DIM, 1> X = VAR.head<STATE_DIM>();
	Eigen::Matrix<CppAD::AD<double>, CONTROL_DIM, 1> U = VAR.segment(STATE_DIM, CONTROL_DIM);
	CppAD::AD<double> T = VAR.tail<1>()(0,0);

	Y(0, 0) += costAD->evaluate(X, U, T);

	functionAD = CppAD::ADFun<double>(VAR, Y);
	functionAD.optimize();
}


template <size_t STATE_DIM, size_t CONTROL_DIM>
void CostFunctionAD<STATE_DIM, CONTROL_DIM>::reverseADTerms(
		std::vector<std::shared_ptr<TermBaseAD>>& termsAD,
		std::vector<std::shared_ptr<CppAD::ADFun<double>>> &functionAD,
		Eigen::Matrix<double, STATE_DIM + CONTROL_DIM +1, 1>& result){

	result.setZero();

	for(size_t termId = 0; termId<termsAD.size(); termId++){
		if (!termsAD[termId]->isActiveAtTime(this->t_)) { continue; }
		result += termsAD[termId]->computeActivation(this->t_) * Reverse1(*functionAD[termId]);
	}
}


template <size_t STATE_DIM, size_t CONTROL_DIM>
Eigen::Matrix<double, STATE_DIM + CONTROL_DIM +1, 1> CostFunctionAD<STATE_DIM, CONTROL_DIM>::Reverse1(CppAD::ADFun<double> &f)
{
	Eigen::Matrix<double, Eigen::Dynamic, 1> dw(STATE_DIM + CONTROL_DIM +1);	// the derivative weighted by the below weights
	Eigen::Matrix<double, Eigen::Dynamic, 1> w(1); 								// weighting vector (of size 1 since a cost function has range dimension 1)
	w << 1.;

	f.Forward(0, var_);		// update the independent variables with current input values (corresponds to function evaluation)
	dw = f.Reverse(1, w);	// compute first order derivative

	return dw;
}
