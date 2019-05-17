/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/


namespace ct {
namespace rbd {

template <size_t ROWS, size_t COLS>
JacobiSingularity<ROWS, COLS>::JacobiSingularity(const Jacobian& J)
{
    J_ = J;
}


template <size_t ROWS, size_t COLS>
double JacobiSingularity<ROWS, COLS>::calc_condition_number() const
{
    SingularValues sv = calc_singular_values();
    return sv.maxCoeff() / sv.minCoeff();
}


template <size_t ROWS, size_t COLS>
double JacobiSingularity<ROWS, COLS>::calc_manipulability(ManipulabilityMethod method) const
{
    double w = 1;

    switch (method)
    {
        case DIRECT:
        {
            // see Siciliano p. 153
            w = std::sqrt((J_ * J_.transpose()).determinant());
            break;
        }
        case SINGULAR_VALUES:
        {
            // can also be calculated by multiplying the singular values of J
            SingularValues sv = calc_singular_values();
            for (int i = 0; i < sv.size(); ++i)
                w *= sv(i);
            break;
        }
    }
    return w;
}


template <size_t ROWS, size_t COLS>
typename JacobiSingularity<ROWS, COLS>::SingularValues JacobiSingularity<ROWS, COLS>::calc_singular_values() const
{
    // matrix is not neccesarily square, so eigenvalues do not exist -> use singular values instead
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_, Eigen::ComputeThinU | Eigen::ComputeThinV);
    return svd.singularValues();
}


template <size_t ROWS, size_t COLS>
typename JacobiSingularity<ROWS, COLS>::JointVelocities JacobiSingularity<ROWS, COLS>::calc_joint_vel(
    const EndeffectorVelocities& des_ee_vel) const
{
    // Use SVD to compute Pseudo Inverse of J = J'(JJ')^-1 = VSU*
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_, Eigen::ComputeThinU | Eigen::ComputeThinV);
    JointVelocities qd_des = svd.solve(des_ee_vel);

    // just for comparison, see if same solution can be reached
    PseudoInverse J_inv = J_.transpose() * (J_ * J_.transpose()).inverse();
    JointVelocities qd_des_pseudo = J_inv * des_ee_vel;

    //assert(qd_des == qd_des_pseudo);

    return qd_des;
}


}  // namespace rbd
}  // namespace ct
