/***********************************************************************************
Copyright (c) 2017, Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo,
Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be used
      to endorse or promote products derived from this software without specific
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/

#ifndef COMMON_MATH_JACOBI_SINGULARITY_H_
#define COMMON_MATH_JACOBI_SINGULARITY_H_

namespace ct {
namespace common {
namespace math {

template<size_t ROWS, size_t COLS>
class JacobiSingularity
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef Eigen::Matrix<double, ROWS, COLS> Jacobian;
  typedef Eigen::Matrix<double, COLS, ROWS> PseudoInverse;
  typedef Eigen::Matrix<double, ROWS, 1> SingularValues;
  typedef Eigen::Matrix<double, ROWS, 1> EndeffectorVelocities;
  typedef Eigen::Matrix<double, COLS, 1> JointVelocities;

  enum ManipulabilityMethod { DIRECT, SINGULAR_VALUES };

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
   * configuration. w(q) = sqrt(det(J*J'))
   *
   * @param method can be calculated directly as described in Siciliano p. 153
   *               or multiplying the singular values of the Jacobian
   * @return the area spanned by all possible ee-velocities.
   */
  double calc_manipulability(ManipulabilityMethod method= DIRECT) const;

  /**
   * Singular values are used if eigenvalues are not available, as in non-
   * square matrices. With Single Value Decomposition (SVD) J = U*S*V.
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
   * J† = J' * (J*J')^(-1) which chooses the "minimum norm" solution out of the
   * pool of infinitely many solutions. (see Siciliano p. 125).
   *
   * @param des_ee_vel desired endeffector velocity
   * @return qd = J† * des_ee_vel
   */
  JointVelocities calc_joint_vel(const EndeffectorVelocities& des_ee_vel) const;

private:
  Jacobian J_;
};


} // namespace math
} // namespace common
} // namespace ct


#include "implementation/JacobiSingularity.h"

#endif /* COMMON_MATH_SINGULARITY_H_ */
