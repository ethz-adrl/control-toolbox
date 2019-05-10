/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/types/StateVector.h>

namespace ct {
namespace rbd {
namespace tpl {

/**
 * \ingroup OS
 * \brief This class provides methods for converting the non-inertia base frame Jacobian matrix to an user
 * defined inertia frame (called as inertia frame).
 */
template <size_t NUM_JOINTS, typename SCALAR>
class FrameJacobian
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Eigen::Matrix<SCALAR, 3, 3> Matrix3s;
    typedef Eigen::Matrix<SCALAR, 3, 1> Vector3s;

    FrameJacobian() {}
    ~FrameJacobian() {}
    /**
	 * This method calculates the inertia frame (frame i) Jacobian (rotational and translation part) given the non-inertia base frame full Jacobian (frame b).
	 *
	 * @param[in] i_R_b		Rotation matrix from base frame to the desired inertia frame.
	 * @param[in] b_r_point	Position of the desired inertia frame origin expressed in the base frame.
	 * @param[in] b_J_point A 6-by-NUM_JOINTS Jacobian matrix (rotational and translation part) expressed in the base frame.
	 * @param[out] i_J_point A 6-by-(6+NUM_JOINTS) Jacobian matrix (rotational and linear translation) expressed in the desired inertia frame.
	 */
    static void FromBaseJacobianToInertiaJacobian(const Matrix3s& i_R_b,
        const Vector3s& b_r_point,
        const Eigen::Matrix<SCALAR, 6, NUM_JOINTS>& b_J_point,
        Eigen::Matrix<SCALAR, 6, NUM_JOINTS + 6>& i_J_point)
    {
        // orientation
        Eigen::Matrix<SCALAR, 3, NUM_JOINTS + 6> i_J_poitOrientation;
        FromBaseJacToInertiaJacOrientation(i_R_b, b_r_point, b_J_point.template topRows<3>(), i_J_poitOrientation);
        // translation
        Eigen::Matrix<SCALAR, 3, NUM_JOINTS + 6> i_J_poitTranslation;
        FromBaseJacToInertiaJacTranslation(i_R_b, b_r_point, b_J_point.template bottomRows<3>(), i_J_poitTranslation);

        i_J_point << i_J_poitOrientation, i_J_poitTranslation;
    }

    /**
	 * This method calculates the inertia frame (frame i) rotational Jacobian given the non-inertia frame rotational Jacobian (frame b).
	 *
	 * @param[in] i_R_b		Rotation matrix from base frame to the desired inertia frame.
	 * @param[in] b_r_point	Position of the desired inertia frame origin expressed in the base frame.
	 * @param[in] b_J_point A 3-by-NUM_JOINTS rotational Jacobian matrix expressed in the base frame.
	 * @param[out] i_J_point A 3-by-(6+NUM_JOINTS) rotational Jacobian matrix expressed in the desired inertia frame.
	 */
    static void FromBaseJacToInertiaJacOrientation(const Matrix3s& i_R_b,
        const Vector3s& b_r_point,
        const Eigen::Matrix<SCALAR, 3, NUM_JOINTS>& b_J_point,
        Eigen::Matrix<SCALAR, 3, NUM_JOINTS + 6>& i_J_point)
    {
        i_J_point << i_R_b, Matrix3s::Zero(), i_R_b * b_J_point;
    }

    /**
	 * This method calculates the inertia frame (frame i) translational Jacobian given the non-inertia base frame translation Jacobian (frame b).
	 *
	 * @param[in] i_R_b		Rotation matrix from base frame to the desired inertia frame.
	 * @param[in] b_r_point	Position of the desired inertia frame origin expressed in the base frame.
	 * @param[in] b_J_point A 3-by-NUM_JOINTS translational Jacobian matrix expressed in the base frame.
	 * @param[out] i_J_point A 3-by-(6+NUM_JOINTS) translational Jacobian matrix expressed in the desired inertia frame.
	 */
    static void FromBaseJacToInertiaJacTranslation(const Matrix3s& i_R_b,
        const Vector3s& b_r_point,
        const Eigen::Matrix<SCALAR, 3, NUM_JOINTS>& b_J_point,
        Eigen::Matrix<SCALAR, 3, NUM_JOINTS + 6>& i_J_point)
    {
        i_J_point << -i_R_b * CrossProductMatrix(b_r_point), i_R_b, i_R_b * b_J_point;
    }


    /**
	 * This method calculates the time derivative of the inertia frame (frame i) Jacobian (rotational and translation part) given the non-inertia base
	 * frame Jacobian (frame b).
	 *
	 * @param[in] generalizedVelocities Generalized coordinate velocities.
	 * @param[in] i_R_b			Rotation matrix from base frame to the desired inertia frame.
	 * @param[in] b_r_point		Position of the desired inertia frame origin expressed in the base frame.
	 * @param[in] b_J_point 	A 6-by-NUM_JOINTS Jacobian matrix (rotational and translation part) expressed in the base frame.
	 * @param[in] b_dJdt_point 	A 6-by-NUM_JOINTS Jacobian matrix time derivative (rotational and translation part) expressed in the base frame.
	 * @param[out] i_dJdt_point A 6-by-(6+NUM_JOINTS) Jacobian matrix time derivative (rotational and translation part) expressed in the desired inertia frame.
	 */
    static void FromBaseJacobianDevToInertiaJacobianDev(
        const Eigen::Matrix<SCALAR, NUM_JOINTS + 6, 1>& generalizedVelocities,
        const Matrix3s& i_R_b,
        const Vector3s& b_r_point,
        const Eigen::Matrix<SCALAR, 6, NUM_JOINTS>& b_J_point,
        const Eigen::Matrix<SCALAR, 6, NUM_JOINTS>& b_dJdt_point,
        Eigen::Matrix<SCALAR, 6, NUM_JOINTS + 6>& i_dJdt_point)
    {
        // orientation
        Eigen::Matrix<SCALAR, 3, NUM_JOINTS + 6> i_dJdt_pointOrientation;
        FromBaseJacobianDevToInertiaJacobianDevOrientation(generalizedVelocities, i_R_b, b_r_point,
            b_J_point.template topRows<3>(), b_dJdt_point.template topRows<3>(), i_dJdt_pointOrientation);
        // translation
        Eigen::Matrix<SCALAR, 3, NUM_JOINTS + 6> i_dJdt_pointTranslation;
        FromBaseJacobianDevToInertiaJacobianDevTranslation(generalizedVelocities, i_R_b, b_r_point,
            b_J_point.template bottomRows<3>(), b_dJdt_point.template bottomRows<3>(), i_dJdt_pointTranslation);

        i_dJdt_point << i_dJdt_pointOrientation, i_dJdt_pointTranslation;
    }

    /**
	 * This method calculates the time derivative of the inertia frame (frame i) rotational Jacobian given the non-inertia base
	 * frame rotational Jacobian (frame b).
	 *
	 * @param[in] generalizedVelocities Generalized coordinate velocities.
	 * @param[in] i_R_b			Rotation matrix from base frame to the desired inertia frame.
	 * @param[in] b_r_point		Position of the desired inertia frame origin expressed in the base frame.
	 * @param[in] b_J_point 	A 6-by-NUM_JOINTS rotational Jacobian matrix expressed in the base frame.
	 * @param[in] b_dJdt_point 	A 6-by-NUM_JOINTS rotational Jacobian matrix time derivative expressed in the base frame.
	 * @param[out] i_dJdt_point A 6-by-(6+NUM_JOINTS) rotational Jacobian matrix time derivative expressed in the desired inertia frame.
	 */
    static void FromBaseJacobianDevToInertiaJacobianDevOrientation(
        const Eigen::Matrix<SCALAR, NUM_JOINTS + 6, 1>& generalizedVelocities,
        const Matrix3s& i_R_b,
        const Vector3s& b_r_point,
        const Eigen::Matrix<SCALAR, 3, NUM_JOINTS>& b_J_point,
        const Eigen::Matrix<SCALAR, 3, NUM_JOINTS>& b_dJdt_point,
        Eigen::Matrix<SCALAR, 3, NUM_JOINTS + 6>& i_dJdt_point)
    {
        // rotation matrix time derivatives
        Matrix3s i_dRdt_b = i_R_b * CrossProductMatrix(generalizedVelocities.template head<3>());
        // orientation
        i_dJdt_point << i_dRdt_b, Matrix3s::Zero(), i_dRdt_b * b_J_point + i_R_b * b_dJdt_point;
    }

    /**
	 * This method calculates the time derivative of the inertia frame (frame i) translational Jacobian given the non-inertia base
	 * frame rotational Jacobian (frame b).
	 *
	 * @param[in] generalizedVelocities Generalized coordinate velocities.
	 * @param[in] i_R_b			Rotation matrix from base frame to the desired inertia frame.
	 * @param[in] b_r_point		Position of the desired inertia frame origin expressed in the base frame.
	 * @param[in] b_J_point 	A 6-by-NUM_JOINTS translational Jacobian matrix expressed in the base frame.
	 * @param[in] b_dJdt_point 	A 6-by-NUM_JOINTS translational Jacobian matrix time derivative expressed in the base frame.
	 * @param[out] i_dJdt_point A 6-by-(6+NUM_JOINTS) translational Jacobian matrix time derivative expressed in the desired inertia frame.
	 */
    static void FromBaseJacobianDevToInertiaJacobianDevTranslation(
        const Eigen::Matrix<SCALAR, NUM_JOINTS + 6, 1>& generalizedVelocities,
        const Matrix3s& i_R_b,
        const Vector3s& b_r_point,
        const Eigen::Matrix<SCALAR, 3, NUM_JOINTS>& b_J_point,
        const Eigen::Matrix<SCALAR, 3, NUM_JOINTS>& b_dJdt_point,
        Eigen::Matrix<SCALAR, 3, NUM_JOINTS + 6>& i_dJdt_point)
    {
        // velocity of the point in the base frame
        Vector3s b_v_point = b_J_point * generalizedVelocities.template tail<NUM_JOINTS>();
        // rotation matrix time derivatives
        Matrix3s i_dRdt_b = i_R_b * CrossProductMatrix(generalizedVelocities.template head<3>());
        // translation
        i_dJdt_point << -i_dRdt_b * CrossProductMatrix(b_r_point) - i_R_b * CrossProductMatrix(b_v_point), i_dRdt_b,
            i_dRdt_b * b_J_point + i_R_b * b_dJdt_point;
    }

private:
    /*
	 * calculates the skew matrix for vector cross product
	 */
    template <typename Derived>
    static Matrix3s CrossProductMatrix(const Eigen::DenseBase<Derived>& in)
    {
        if (in.innerSize() != 3 || in.outerSize() != 1)
            throw std::runtime_error("Input argument should be a 3-by-1 vector.");

        Matrix3s out;
        out << SCALAR(0.0), -in(2), +in(1), +in(2), SCALAR(0.0), -in(0), -in(1), +in(0), SCALAR(0.0);
        return out;
    }
};

}  // namespace tpl

template <size_t NUM_JOINTS>
using FrameJacobian = tpl::FrameJacobian<NUM_JOINTS, double>;

}  // namespace rbd
}  // namespace ct
