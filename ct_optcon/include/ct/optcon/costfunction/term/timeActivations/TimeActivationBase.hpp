/*
 * TimeActivationBase.hpp
 *
 *  Created on: Feb 10, 2017
 *      Author: neunertm
 */

#ifndef INCLUDE_CT_OPTCON_COSTFUNCTION_TERM_TIMEACTIVATIONS_TIMEACTIVATIONBASE_HPP_
#define INCLUDE_CT_OPTCON_COSTFUNCTION_TERM_TIMEACTIVATIONS_TIMEACTIVATIONBASE_HPP_

namespace ct {
namespace optcon {

namespace tpl {

template <typename SCALAR>
class TimeActivationBase
{
public:
	TimeActivationBase(){}
	virtual ~TimeActivationBase() {}

	virtual void loadConfigFile(const std::string& filename, const std::string& termName, bool verbose = false){ 
		return;
	} 

	virtual bool isActiveAtTime(const SCALAR t) { return true; }

	// virtual 

	virtual SCALAR computeActivation(const SCALAR t) { return SCALAR(1.0); }

	virtual void printInfo()
	{
		std::cout << "Cost Function active at all times" << std::endl;
	}
};

}

typedef tpl::TimeActivationBase<double> TimeActivationBase;

}
}



#endif /* INCLUDE_CT_OPTCON_COSTFUNCTION_TERM_TIMEACTIVATIONS_TIMEACTIVATIONBASE_HPP_ */
