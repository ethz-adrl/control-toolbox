/*
 * SingleActivation.hpp
 *
 *  Created on: Feb 10, 2017
 *      Author: neunertm
 */

#ifndef INCLUDE_CT_OPTCON_COSTFUNCTION_TERM_TIMEACTIVATIONS_RBFGAUSSACTIVATION_HPP_
#define INCLUDE_CT_OPTCON_COSTFUNCTION_TERM_TIMEACTIVATIONS_RBFGAUSSACTIVATION_HPP_

#include <cmath>
#include <math.h>
#include "TimeActivationBase.hpp"

namespace ct {
namespace optcon {
namespace tpl {

template <typename SCALAR>
class RBFGaussActivation : public TimeActivationBase<SCALAR>
{
public:
	RBFGaussActivation(){}

	RBFGaussActivation(const SCALAR mu, const SCALAR sigma) :
		mu_(mu),
		sigma_(sigma)
	{
		sigma2inv_ = 1 / (2 * sigma_ * sigma_);
		factor_ = 1.0 / sqrt(2.0 * M_PI * sigma_ * sigma_); 
	}
	virtual ~RBFGaussActivation() {}

	RBFGaussActivation(const RBFGaussActivation& arg): 
	mu_(arg.mu_),
	sigma_(arg.sigma_),
	sigma2inv_(arg.sigma2inv_),
	factor_(arg.factor_)
	{}

	virtual void loadConfigFile(const std::string& filename, const std::string& termName, bool verbose = false) override {
		boost::property_tree::ptree pt;
		boost::property_tree::read_info(filename, pt); 
		mu_ = pt.get<SCALAR>(termName + ".mu");
		sigma_ = pt.get<SCALAR>(termName + ".sigma");
		// factors used for efficient computeActivation calculation
		sigma2inv_ = - 1.0 / (2.0 * sigma_ * sigma_);
		factor_ = 1.0 / sqrt(2.0 * M_PI * sigma_ * sigma_); 
	} 

	virtual bool isActiveAtTime(const SCALAR t) override {
		return true;
	}

	virtual SCALAR computeActivation(const SCALAR t) override {
	 	return factor_ * exp((t - mu_)*(t - mu_) * sigma2inv_); 
	}

	virtual void printInfo() override
	{
		std::cout << "RBF costfunction time activation" << std::endl;
		std::cout << "mu: " << mu_ << "s \t sigma: " << sigma_ << "s" << std::endl;
	}

private:
	SCALAR mu_;
	SCALAR sigma_;
	SCALAR sigma2inv_;
	SCALAR factor_;
};

}

typedef tpl::RBFGaussActivation<double> RBFGaussActivation;

}
}


#endif /* INCLUDE_CT_OPTCON_COSTFUNCTION_TERM_TIMEACTIVATIONS_RBFGAUSSACTIVATION_HPP_ */
