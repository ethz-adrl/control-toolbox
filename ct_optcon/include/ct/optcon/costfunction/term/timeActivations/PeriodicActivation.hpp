/*
 * PeriodicActivation.hpp
 *
 *  Created on: Feb 10, 2017
 *      Author: neunertm
 */

#ifndef INCLUDE_CT_OPTCON_COSTFUNCTION_TERM_TIMEACTIVATIONS_PERIODICACTIVATION_HPP_
#define INCLUDE_CT_OPTCON_COSTFUNCTION_TERM_TIMEACTIVATIONS_PERIODICACTIVATION_HPP_

#include <cmath>
#include "TimeActivationBase.hpp"

namespace ct {
namespace optcon {

class PeriodicActivation : public TimeActivationBase
{
public:
	PeriodicActivation(){}

	PeriodicActivation(
			const double active_percentage,
			const double period,
			const double activation_offset,
			const double period_offset) :
		active_percentage_(active_percentage),
		period_(period),
		activation_offset_(activation_offset),
		period_offset_(period_offset)
	{}
	virtual ~PeriodicActivation() {}

	PeriodicActivation(const PeriodicActivation& arg): 
	active_percentage_(arg.active_percentage_),
	period_(arg.period_),
	activation_offset_(arg.activation_offset_),
	period_offset_(arg.period_offset_),
	t_end_(arg.t_end_)
	{}

	virtual void loadConfigFile(const std::string& filename, const std::string& termName, bool verbose = false) override 
	{
		boost::property_tree::ptree pt;
		boost::property_tree::read_info(filename, pt); 

		active_percentage_ = pt.get<double>(termName + ".active_percentage");
		period_ = pt.get<double>(termName + ".period");
		activation_offset_ = pt.get<double>(termName + ".activation_offset");
		period_offset_ = pt.get<double>(termName + ".period_offset");
		t_end_ = pt.get<double>(termName + ".t_end");

		if(activation_offset_ + active_percentage_ * period_ > period_)
		{
			throw std::runtime_error("Activation offset plus active period percentage exceed period time. Adjust settings"); 
		}
	} 

	// to verify
	virtual bool isActiveAtTime(const double t) override {
		bool active = false;
		if(t >= period_offset_ && t < t_end_)
		{
			double t0 = t - period_offset_;
			double t0norm = std::fmod(t0, period_);
			if(t0norm >= activation_offset_ && t0norm < (activation_offset_ + active_percentage_ * period_))
				active = true;
		}
		return active;
	}

	virtual double computeActivation(const double t) override { return 1.0; }

	virtual void printInfo() override
	{
		std::cout << "Cost function active at periodic times: " <<  std::endl;
		std:: cout << "Period: " << period_ << "\nOffset after period start:  "  << activation_offset_ << "s" << std::endl;
		std::cout << "Offset between t0: "  << period_offset_ << "s" << std::endl;
		std::cout << "Active for " << 100*active_percentage_ << "% of the period" << std::endl;
	}

private:
	double active_percentage_; // how much of the cycle is the time active
	double period_; // what is the period
	double activation_offset_; // how much is the activation offset WITHIN the period
	double period_offset_; // how much is the period offset to t=0?
	double t_end_;
};

}
}




#endif /* INCLUDE_CT_OPTCON_COSTFUNCTION_TERM_TIMEACTIVATIONS_PERIODICACTIVATION_HPP_ */
