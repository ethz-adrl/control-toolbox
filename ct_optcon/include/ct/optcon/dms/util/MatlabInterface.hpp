#ifndef DMS_MATLAB_INTERFACE_HPP_
#define DMS_MATLAB_INTERFACE_HPP_

#ifdef MATLAB

#include <ct/optcon/matlab.hpp>
#include <Eigen/Dense>


class MatlabInterface{

public:

	MatlabInterface(std::string pathToLocation = "~/")
{
		matfile_.open(pathToLocation, matlab::MatFile::WRITE_COMPRESSED);
}

	~MatlabInterface(){
		matfile_.close();
	}

	void sendScalarArrayToMatlab(std::vector<double, Eigen::aligned_allocator<double>> T, const std::string var_name)
	{
		size_t traj_length = T.size();

		Eigen::VectorXd trajectory;
		trajectory.resize(traj_length);
		for(size_t i = 0; i< traj_length; i++)
		{
			trajectory(i) = T[i];
		}
		assert(matfile_.isOpen());
		assert(matfile_.isWritable());
		matfile_.put(var_name, trajectory);
	}

	void sendScalarTrajectoryToMatlab(const Eigen::VectorXi& trajectory, const std::string var_name)
	{
		Eigen::VectorXd traj = trajectory.cast <double>();
		assert(matfile_.isOpen());
		assert(matfile_.isWritable());
		matfile_.put(var_name, traj);
	}
	void sendScalarTrajectoryToMatlab1(const Eigen::VectorXd& trajectory, const std::string var_name)
	{
		matfile_.put(var_name, trajectory);
	}


	template <typename TRAJ_TYPE>
	void sendMultiDimTrajectoryToMatlab(TRAJ_TYPE T, const size_t element_dim, const std::string name) //introducing element-dim here is a hack... @ Michael .better way to do it?
	{
		const size_t traj_length = T.size();

		Eigen::MatrixXd trajectory;
		trajectory.resize(element_dim, traj_length);

		for(size_t i = 0; i< traj_length; i++)
		{
			trajectory.col(i) = (Eigen::VectorXd) T[i];
		}
		assert(matfile_.isOpen());
		assert(matfile_.isWritable());
		matfile_.put(name, trajectory);
	}


private:
	matlab::MatFile matfile_;


};

#endif // MATLAB

#endif //DMS_MATLAB_INTERFACE_HPP_
