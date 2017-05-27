#ifndef DMS_STABILISER_TASK_COMMON_HPP_
#define DMS_STABILISER_TASK_COMMON_HPP_

#include <ct/optcon/dms/dms_core/DmsSettings.hpp>

namespace ct {
namespace optcon {


typedef enum StabiliserType
{
	TVLQR = 0,
	ILQG = 1,

	num_types_stabiliser
} StabiliserType_t;



struct stab_settings
{
	StabiliserType_t stabiliserType_ = TVLQR;
	DmsSettings::SplineType_t stateSpliner_ = DmsSettings::PIECEWISE_LINEAR;
	DmsSettings::SplineType_t inputSpliner_ = DmsSettings::ZERO_ORDER_HOLD;
	double dtStab_ = 0.02;
	size_t nThreads_ = 2; 
	size_t useJointPDController_ = true;
};


void printStabilisationSettings(stab_settings settings)
{
	std::cout<<"============================================================="<<std::endl;
	std::cout<<"\tStab Settings: "<<std::endl;
	std::cout<<"============================================================="<<std::endl;

	switch(settings.stabiliserType_)
	{
		case TVLQR:
		{
			std::cout << "Using TVLQR Stabiliser" << std::endl;
			break;
		}
		case ILQG:
		{
			std::cout << "Using ILQG Stabiliser" << std::endl;
			break;
		}

		default:
		{
			throw(std::runtime_error("Unknown Stabiliser Type, Exiting"));
			break;
		}
	}

	switch(settings.stateSpliner_)
	{
		case DmsSettings::ZERO_ORDER_HOLD:
		{
			std::cout << "Using ZOH Stab State Spliner Stabiliser" << std::endl;
			break;
		}
		case DmsSettings::PIECEWISE_LINEAR:
		{
			std::cout << "Using PWL Stab State Spliner Stabiliser" << std::endl;
			break;
		}

		default:
		{
			throw(std::runtime_error("Unknown State Stab Type, Exiting"));
			break;
		}
	}	

	switch(settings.inputSpliner_)
	{
		case DmsSettings::ZERO_ORDER_HOLD:
		{
			std::cout << "Using ZOH Stab Control Spliner Stabiliser" << std::endl;
			break;
		}
		case DmsSettings::PIECEWISE_LINEAR:
		{
			std::cout << "Using PWL Stab Control Spliner Stabiliser" << std::endl;
			break;
		}

		default:
		{
			throw(std::runtime_error("Unknown Control Stab Type, Exiting"));
			break;
		}
	}		

	std::cout<<"Timestep Stab: " << settings.dtStab_ << "s" <<std::endl;
	std::cout << "Number of CPU Threads: " << settings.nThreads_ << std::endl;

	std::cout<<"============================================================="<<std::endl;
}

} // namespace optcon
} // namespace ct

#endif
