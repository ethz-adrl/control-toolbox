/*
 * SingleActivation.hpp
 *
 *  Created on: Feb 10, 2017
 *      Author: neunertm
 */

#ifndef INCLUDE_CT_OPTCON_COSTFUNCTION_TERM_TIMEACTIVATIONS_SINGLEACTIVATION_HPP_
#define INCLUDE_CT_OPTCON_COSTFUNCTION_TERM_TIMEACTIVATIONS_SINGLEACTIVATION_HPP_

#include "TimeActivationBase.hpp"

namespace ct {
namespace optcon {

class SingleActivation : public TimeActivationBase
{
public:
	SingleActivation(){}

	SingleActivation(const double t_on, const double t_off) :
		t_on_(t_on),
		t_off_(t_off)
	{}
	virtual ~SingleActivation() {}

	SingleActivation(const SingleActivation& arg): 
	t_on_(arg.t_on_),
	t_off_(arg.t_off_)
	{}

	virtual void loadConfigFile(const std::string& filename, const std::string& termName, bool verbose = false) override {
		boost::property_tree::ptree pt;
		boost::property_tree::read_info(filename, pt); 
		t_on_ = pt.get<double>(termName + ".t_on");
		t_off_ = pt.get<double>(termName + ".t_off");
	} 

	virtual bool isActiveAtTime(const double t) override {
		return (t >= t_on_ && t<t_off_);
	}

	virtual double computeActivation(const double t) override { return 1.0; }

	virtual void printInfo() override
	{
		std::cout << "Cost Function active between time: " << t_on_ << "s and time: " << t_off_ << "s" << std::endl;
	}

private:
	double t_on_;
	double t_off_;
};

}
}


#endif /* INCLUDE_CT_OPTCON_COSTFUNCTION_TERM_TIMEACTIVATIONS_SINGLEACTIVATION_HPP_ */
