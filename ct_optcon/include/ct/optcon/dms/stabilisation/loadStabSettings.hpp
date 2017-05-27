#ifndef DMS_LOAD_STABILISER_TASK_HPP_
#define DMS_LOAD_STABILISER_TASK_HPP_

#include "StabilisationCommon.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>

namespace ct {
namespace optcon {

void loadStabilisationSettings(const std::string& filename, stab_settings& settings)
{
	boost::property_tree::ptree pt;
	boost::property_tree::read_info(filename, pt);

	settings.stabiliserType_ = (StabiliserType_t) pt.get<unsigned int>("stab.stabiliserType");
	settings.dtStab_ = pt.get<double>("stab.dt_stab");
	settings.stateSpliner_ = (DmsSettings::SplineType_t) pt.get<unsigned int>("stab.stateSpliner");
	settings.inputSpliner_ = (DmsSettings::SplineType_t) pt.get<unsigned int>("stab.inputSpliner");
	settings.nThreads_ = pt.get<unsigned int>("stab.nThreads");
	settings.useJointPDController_ = pt.get<unsigned int>("stab.useJointPDController");
}

} // namespace optcon
} // namespace ct

#endif
