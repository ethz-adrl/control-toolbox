#ifndef DMS_LOAD_OBSTACLE_HPP_
#define DMS_LOAD_OBSTACLE_HPP_

#include <Eigen/Dense>
#include <ct/optcon/dms/robotics_plugin/Obstacle3d.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>

namespace ct {
namespace optcon {

void loadEllipsoidObstacle(
	const std::string& filename,
	const size_t index, 
	std::vector<std::shared_ptr<Obstacle3d>>& obstacles
	)
{
	std::string obstName = "obstacleEllipsoid" + std::to_string(index);
	std::cout << "Obstacle Name: " << obstName << std::endl;

	boost::property_tree::ptree pt;
	boost::property_tree::read_info(filename, pt);

	Eigen::Vector3d ctr;
	Eigen::Vector3d ha;
	double rotX;
	double rotY;
	double rotZ;
	Eigen::Vector3d vel;
	Eigen::Vector3d rt;

	loadMatrixDms(filename, obstName + ".ctr", ctr);
	loadMatrixDms(filename, obstName + ".halfAxes", ha);
	loadMatrixDms(filename, obstName + ".velocity", vel);
	loadMatrixDms(filename, obstName + ".rates", rt);

	rotX = pt.get<double>(obstName + ".rotation.x");
	rotY = pt.get<double>(obstName + ".rotation.y");
	rotZ = pt.get<double>(obstName + ".rotation.z");
	Eigen::Matrix3d rot;
	rot = Eigen::AngleAxisd(rotX, Eigen::Vector3d::UnitX())
	* Eigen::AngleAxisd(rotY, Eigen::Vector3d::UnitY())
	* Eigen::AngleAxisd(rotZ, Eigen::Vector3d::UnitZ());

	Eigen::Quaterniond q(rot);

	std::cout << "Quaterion: " << std::endl;// << q << std::endl;

	std::shared_ptr<Obstacle3d> obst;

	obst = std::shared_ptr<Obstacle3d> (new Obstacle3d(Ellipsoidal3d, ctr, ha, q, vel, rt));
	obstacles.push_back(obst);
}

void loadCylindricalObstacle(
	const std::string& filename,
	const size_t index,
	std::vector<std::shared_ptr<Obstacle3d>>& obstacles
	)
{
	std::string obstName = "obstacleCylinderSphere" + std::to_string(index);
	std::cout << "Obstacle Name: " << obstName << std::endl;

	boost::property_tree::ptree pt;
	boost::property_tree::read_info(filename, pt);

	Eigen::Vector3d ctr;	
	Eigen::Matrix3d selectionMatrix;
	double radius;
	double rotX;
	double rotY;
	double rotZ;
	Eigen::Vector3d vel;
	Eigen::Vector3d rt;

	loadMatrixDms(filename, obstName + ".selectionMatrix", selectionMatrix);
	loadMatrixDms(filename, obstName + ".velocity", vel);
	loadMatrixDms(filename, obstName + ".rates", rt);
	loadMatrixDms(filename, obstName + ".ctr", ctr);

	radius = pt.get<double>	(obstName + ".radius");
	rotX = pt.get<double>	(obstName + ".rotation.x");
	rotY = pt.get<double>	(obstName + ".rotation.y");
	rotZ = pt.get<double>	(obstName + ".rotation.z");
	Eigen::Matrix3d rot;
	rot = Eigen::AngleAxisd(rotX, Eigen::Vector3d::UnitX())
	* Eigen::AngleAxisd(rotY, Eigen::Vector3d::UnitY())
	* Eigen::AngleAxisd(rotZ, Eigen::Vector3d::UnitZ());
	Eigen::Quaterniond q(rot);

	std::shared_ptr<Obstacle3d> obst;

	obst = std::shared_ptr<Obstacle3d> (new Obstacle3d(SimpleCylinderOrSphere3d, ctr, selectionMatrix, radius, q, vel, rt));
	obstacles.push_back(obst);
}

void loadRbdSettings(
	const std::string& filename,
	dms_robotics_settings& settings)
{
	boost::property_tree::ptree pt;
	boost::property_tree::read_info(filename, pt);

	settings.costType_ = (CostType_t) pt.get<unsigned int>("rbd.CostType");
	settings.nrObstEllipsoid_ = pt.get<unsigned int>("rbd.NrObstEllipsoid");
	settings.nrObstSphere_ =  pt.get<unsigned int>("rbd.NrObstSphere");
}

} // namespace optcon
} // namespace ct


#endif /* LOADTASK_HPP_ */
