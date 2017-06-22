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


namespace ct {
namespace rbd {

template<size_t ROWS, size_t COLS>
JacobiSingularity<ROWS,COLS>::JacobiSingularity(const Jacobian& J)
{
  J_ = J;
}


template<size_t ROWS, size_t COLS>
double
JacobiSingularity<ROWS,COLS>::calc_condition_number() const
{
  SingularValues sv = calc_singular_values();
  return sv.maxCoeff()/sv.minCoeff();
}


template<size_t ROWS, size_t COLS>
double
JacobiSingularity<ROWS,COLS>::calc_manipulability(ManipulabilityMethod method) const
{
  double w = 1;

  switch (method) {
  case DIRECT: {
    // see Siciliano p. 153
    w = std::sqrt( (J_ * J_.transpose()).determinant() );
    break;
  }
  case SINGULAR_VALUES: {
    // can also be calculated by multiplying the singular values of J
    SingularValues sv = calc_singular_values();
    for (int i = 0; i < sv.size(); ++i)
      w *= sv(i);
    break;
  }
  }
  return w;
}


template<size_t ROWS, size_t COLS>
typename JacobiSingularity<ROWS,COLS>::SingularValues
JacobiSingularity<ROWS,COLS>::calc_singular_values() const
{
  // matrix is not neccesarily square, so eigenvalues do not exist -> use singular values instead
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_, Eigen::ComputeThinU | Eigen::ComputeThinV);
  return svd.singularValues();
}


template<size_t ROWS, size_t COLS>
typename JacobiSingularity<ROWS,COLS>::JointVelocities
JacobiSingularity<ROWS,COLS>::calc_joint_vel(const EndeffectorVelocities& des_ee_vel) const
{
  // Use SVD to compute Pseudo Inverse of J = J'(JJ')^-1 = VSU*
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_, Eigen::ComputeThinU | Eigen::ComputeThinV);
  JointVelocities qd_des = svd.solve(des_ee_vel);

  // just for comparison, see if same solution can be reached
  PseudoInverse J_inv = J_.transpose() * (J_*J_.transpose()).inverse();
  JointVelocities qd_des_pseudo = J_inv * des_ee_vel;

  //assert(qd_des == qd_des_pseudo);

  return qd_des;
}


} // namespace rbd
} // namespace ct

