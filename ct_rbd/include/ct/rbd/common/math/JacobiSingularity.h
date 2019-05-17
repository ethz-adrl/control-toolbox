/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace rbd {

/*!
 * \brief A class computing codition numbers and singular velues of a Jacobian
 *
 * \tparam ROWS number of rows
 * \tparam CLOS number of colums
 */
template <size_t ROWS, size_t COLS>
class JacobiSingularity
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Eigen::Matrix<double, ROWS, COLS> Jacobian;
    typedef Eigen::Matrix<double, COLS, ROWS> PseudoInverse;
    typedef Eigen::Matrix<double, ROWS, 1> SingularValues;
    typedef Eigen::Matrix<double, ROWS, 1> EndeffectorVelocities;
    typedef Eigen::Matrix<double, COLS, 1> JointVelocities;

    enum ManipulabilityMethod
    {
        DIRECT,
        SINGULAR_VALUES
    };

public:
    JacobiSingularity(const Jacobian& J);

    /**
   * The condition number compares the lowest to the highest eigenvalue (for
   * square matrix) or singular value (for others).
   *
   * High values of ev_max/ev_min imply that some inputs ("null-space") are mapped to
   * values close to zero outputs, e.g. some actions in operational space are restricted.
   */
    double calc_condition_number() const;

    /**
   * The manipulability describes the freedom of the endeffector for a given
   * configuration. \f$ w(q) = \sqrt(det(J*J')) \f$
   *
   * @param method can be calculated directly as described in Siciliano p. 153
   *               or multiplying the singular values of the Jacobian
   * @return the area spanned by all possible ee-velocities.
   */
    double calc_manipulability(ManipulabilityMethod method = DIRECT) const;

    /**
   * Singular values are used if eigenvalues are not available, as in non-
   * square matrices. With Single Value Decomposition (SVD) \f$ J = U S V \f$.
   * See Siciliano p. 577
   * @return the diagonals in S
   */
    SingularValues calc_singular_values() const;

    /**
   * Calculates the joint velocity to obtain a specified endeffector velocity.
   *
   * In general the Jacobian is not a square matrix, so the inverse is not
   * defined since there exist either no solution (skinny Matrix) or infinitely
   * many (fat Matrix). This function computes the pseudo inverse
   * \f$ J^\dagger = \f$J'  (J J')^{-1} \f$ which chooses the "minimum norm" solution out of the
   * pool of infinitely many solutions. (see Siciliano p. 125).
   *
   * @param des_ee_vel desired endeffector velocity
   * @return qd = J† * des_ee_vel
   */
    JointVelocities calc_joint_vel(const EndeffectorVelocities& des_ee_vel) const;

private:
    Jacobian J_;
};


}  // namespace rbd
}  // namespace ct


#include "implementation/JacobiSingularity.h"
