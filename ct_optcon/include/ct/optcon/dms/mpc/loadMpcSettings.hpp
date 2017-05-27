#ifndef DMS_MPC_LOAD_MPC_SETTINGS_HPP_
#define DMS_MPC_LOAD_MPC_SETTINGS_HPP_

#include <ct/optcon/mpc/MpcSettings.h>

namespace ct {
namespace optcon {


//void loadReplanSettings(const std::string& filename, mpc_settings& settings)
//{
//	boost::property_tree::ptree pt;
//	boost::property_tree::read_info(filename, pt);
//
//	settings.maxIterations_ = pt.get<int>("replanning.maxIterations");
//	settings.stateForwardIntegration_ = pt.get<bool>("replanning.stateForwardIntegration");
//	settings.shrinkTimeHorizon_ = pt.get<bool>("replanning.shrinkTimeHorizon");
//	settings.measureDelay_ = pt.get<bool>("replanning.measureDelay");
//	settings.fixedDelayUs_ = pt.get<int>("replanning.fixedDelayUs");
//	settings.timeHorizon_ = pt.get<double>("replanning.timeHorizon");
//	settings.additionalDelayUs_ = pt.get<double>("replanning.additionalDelayUs");
//	settings.coldStart_ = pt.get<bool>("replanning.coldStart");
//	settings.solverType_ = (SolverType_t) pt.get<int>("replanning.SolverType");
//}

} // namespace optcon
} // namespace ct

#endif
