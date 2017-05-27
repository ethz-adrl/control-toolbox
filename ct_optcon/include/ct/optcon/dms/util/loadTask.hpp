#ifndef DMS_LOADTASK_HPP_
#define DMS_LOADTASK_HPP_

#include <Eigen/Dense>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>

namespace ct {
namespace optcon {


template <typename Derived>
void loadMatrixDms(const std::string& filename, const std::string& matrixName, Eigen::MatrixBase<Derived>& matrix)
{
	size_t rows = matrix.rows();
	size_t cols = matrix.cols();
	matrix.setZero();

	boost::property_tree::ptree pt;
	boost::property_tree::read_info(filename, pt);

	double scaling = pt.get<double>(matrixName + ".scaling", 1.0);

	for (size_t i=0; i<rows; i++)
	{
		for (size_t j=0; j<cols; j++)
		{
			matrix(i,j) = scaling*pt.get<double>(matrixName + "." + "(" +std::to_string(i) + "," + std::to_string(j) + ")" , 0.0);
		}
	}
}


void loadScalar(const std::string& filename, const std::string& scalarName, double& scalar)
{
	boost::property_tree::ptree pt;
	boost::property_tree::read_info(filename, pt);

	scalar = pt.get<double>("T_penalty.scaling");
}


void loadVisualizationSettings(const std::string& filename, double& slowdown)
{
	boost::property_tree::ptree pt;
	boost::property_tree::read_info(filename, pt);

	slowdown = pt.get<double>("visualization.slowdown");
}

//template <typename Derived>
//void loadIntermediateCost(const std::string& filename, Eigen::MatrixBase<Derived>& Q_intermediate, double& sigma, double& tp)
//{
//	loadMatrix(filename, "Q_intermediate", Q_intermediate);
//
//	boost::property_tree::ptree pt;
//	boost::property_tree::read_info(filename, pt);
//
//	sigma = pt.get<double>("sigma");
//	tp = pt.get<double>("tp");
//}

} // namespace optcon
} // namespace ct



#endif /* LOADTASK_HPP_ */
