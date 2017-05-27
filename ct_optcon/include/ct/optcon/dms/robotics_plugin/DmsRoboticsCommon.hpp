#ifndef DMS_ROBOTICS_COMMON_HPP_
#define DMS_ROBOTICS_COMMON_HPP_

#include <iostream>

namespace ct {
namespace optcon {

typedef enum CostType
{
	JOINTSPACE = 0,
	GRAVITY = 1,
	TASKSPACE = 2,

	num_types_costtype
} CostType_t;

struct dms_robotics_settings
{
	CostType_t costType_ = GRAVITY;
	size_t nrObstEllipsoid_ = 0;
	size_t nrObstSphere_ = 0;	
};

void printRbdSettings(dms_robotics_settings settings)
{
	std::cout<<"============================================================="<<std::endl;
	std::cout<<"\tRBD Settings: "<<std::endl;
	std::cout<<"============================================================="<<std::endl;

	switch(settings.costType_)
	{
		case JOINTSPACE:
		{
			std::cout << "Using Jointspace Costfunction" << std::endl;
			break;
		}
		case GRAVITY:
		{
			std::cout << "Using Gravity compensation Costfunction" << std::endl;
			break;
		}
		case TASKSPACE:
		{
			std::cout << "Using Taskspace Costfunction" << std::endl;
			break;
		}

		default:
		{
			throw(std::runtime_error("Unknown Costfunction Type, Exiting"));
			break;
		}
	}	

	std::cout<<"Number of Ellipsoidal Obstacles : " << settings.nrObstEllipsoid_ <<std::endl;
	std::cout<<"Number of Spherical Obstacles: " << settings.nrObstSphere_ << std::endl;

	std::cout<<"============================================================="<<std::endl;
}

} // namespace optcon
} // namespace ct

#endif
