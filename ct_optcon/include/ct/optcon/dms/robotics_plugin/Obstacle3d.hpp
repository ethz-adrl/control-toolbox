#ifndef DMS_OBSTACLE_3D_HPP
#define DMS_OBSTACLE_3D_HPP

namespace ct {
namespace optcon {


typedef enum ObstacleType3d
{
	Ellipsoidal3d = 0,
	SimpleCylinderOrSphere3d = 1
} ObstacleType3d_t;


struct Obstacle3d
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// Constructor for ellipsoidal case
	Obstacle3d(const ObstacleType3d_t ty,
			const Eigen::Vector3d& ctr,
			const Eigen::Vector3d& ha,
			const Eigen::Quaterniond& orient = Eigen::Quaterniond(1,0,0,0),
			const Eigen::Vector3d& vel = Eigen::Vector3d::Zero(),
			const Eigen::Vector3d& rt = Eigen::Vector3d::Zero()
	):
		type_(ty),
		center_(ctr),
		q_(orient),
		velocity_(vel),
		rates_(rt),
		half_axes_(ha)
	{}

	// Constructor for simple cylinder or sphere case
	Obstacle3d(const ObstacleType3d_t ty, const Eigen::Vector3d& ctr,
			 const Eigen::Matrix3d& sl, const double rad,
			const Eigen::Quaterniond& orient = Eigen::Quaterniond(1,0,0,0),
			const Eigen::Vector3d&  vel = Eigen::Vector3d::Zero(),
			const Eigen::Vector3d& rt = Eigen::Vector3d::Zero()
	):
		type_(ty),
		center_(ctr),
		selectionMatrix_(sl),
		radius_(rad),
		q_(orient),
		velocity_(vel),
		rates_(rt)
	{
		half_axes_ = sl*Eigen::Vector3d(rad,rad,rad);
	}


	//Used for feedforward simulations
	Eigen::Vector3d getPosition(double time)
	{
		Eigen::Vector3d newPos = center_;
		newPos += time*velocity_;
		return newPos;
	}

	Eigen::Quaterniond getOrientation(double time)
	{
		Eigen::Matrix3d deltaR(
				Eigen::AngleAxisd(time*rates_(0), Eigen::Vector3d::UnitX()) *
				Eigen::AngleAxisd(time*rates_(1), Eigen::Vector3d::UnitY()) *
				Eigen::AngleAxisd(time*rates_(2), Eigen::Vector3d::UnitZ()));

		return (Eigen::Quaterniond(deltaR) * q_);
	}

	void printObstacle()
	{
		switch(type_)
		{
			case Ellipsoidal3d: 
			{
				std::cout << "I am an Ellipoidal Obstacle" << std::endl;
				break;
			}
			case SimpleCylinderOrSphere3d:
			{
				std::cout << "I am a Spherical Obstacle" << std::endl;
				break;
			}
			default:
			{
				std::cout << "Unknown obstacle type" << std::endl;
			}
		}

		std::cout << "Center: " << center_.transpose() << std::endl;
		std::cout << "Orientation: " << q_.coeffs().transpose() << std::endl;
		std::cout << "Velocity: " << velocity_.transpose() << std::endl;
		std::cout << "Half Axes: " << half_axes_.transpose() << std::endl;
		std::cout << "Rates: " << rates_.transpose() << std::endl;
	}

	const ObstacleType3d_t& type()  const {return type_;}
	ObstacleType3d_t& type() {return type_;}

	const Eigen::Vector3d& center() const {return center_;}
	Eigen::Vector3d& center() {return center_;}

	const Eigen::Matrix3d& selectionMatrix() const {return selectionMatrix_;}
	Eigen::Matrix3d& selectionMatrix() {return selectionMatrix_;}

	const double& radius() const {return radius_;}
	double& radius() {return radius_;}

	const Eigen::Quaterniond& quaternion() const {return q_;}
	Eigen::Quaterniond& quaternion() {return q_;}

	const Eigen::Vector3d& velocity() const {return velocity_;}
	Eigen::Vector3d& velocity(){return velocity_;}

	const Eigen::Vector3d& rates() const {return rates_;}
	Eigen::Vector3d& rates() {return rates_;}

	const Eigen::Vector3d& halfAxes() const {return half_axes_;}
	Eigen::Vector3d& halfAxes(){return half_axes_;}

private:
	ObstacleType3d_t type_;
	Eigen::Vector3d center_;
	Eigen::Matrix3d selectionMatrix_;
	double radius_;
	Eigen::Quaterniond q_;
	Eigen::Vector3d velocity_;
	Eigen::Vector3d rates_;
	Eigen::Vector3d half_axes_;

};

} // namespace optcon
} // namespace ct

#endif
