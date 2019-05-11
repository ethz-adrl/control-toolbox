/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace rbd {

/*!
 * @brief Inverse Kinematics joint limit constraints
 */
template <typename KINEMATICS, typename SCALAR = double>
class JointLimitConstraints final : public ct::optcon::tpl::DiscreteConstraintBase<SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using VectorXs = Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>;
    using JointPosition = typename ct::rbd::JointState<KINEMATICS::NJOINTS, SCALAR>::Position;

    JointLimitConstraints(std::shared_ptr<ct::optcon::tpl::OptVector<SCALAR>> optVector,
        const JointPosition& lowerBound,
        const JointPosition& upperBound)
        : optVector_(optVector)
    {
        lowerBounds_ = lowerBound;
        upperBounds_ = upperBound;
    }

    ~JointLimitConstraints() override = default;

    VectorXs eval() override
    {
        contraints_.setZero();
        contraints_ = optVector_->getOptimizationVars();
        return contraints_;
    }

    VectorXs evalSparseJacobian() override
    {
        jacobian_.setConstant(static_cast<SCALAR>(1.0));
        return jacobian_;
    }

    size_t getConstraintSize() override { return KINEMATICS::NJOINTS; }
    size_t getNumNonZerosJacobian() override { return KINEMATICS::NJOINTS; }
    void genSparsityPattern(Eigen::VectorXi& iRow_vec, Eigen::VectorXi& jCol_vec) override
    {
        iRow_vec.resize(KINEMATICS::NJOINTS);
        jCol_vec.resize(KINEMATICS::NJOINTS);

        for (size_t i = 0; i < KINEMATICS::NJOINTS; i++)
        {
            iRow_vec(i) = i;
            jCol_vec(i) = i;
        }
    }


    void genSparsityPatternHessian(Eigen::VectorXi& iRow_vec, Eigen::VectorXi& jCol_vec) override
    {
        // do nothing
    }

    void sparseHessianValues(const Eigen::VectorXd& optVec,
        const Eigen::VectorXd& lambda,
        Eigen::VectorXd& sparseHes) override
    {
        // do nothing
    }

    VectorXs getLowerBound() override { return lowerBounds_; }
    VectorXs getUpperBound() override { return upperBounds_; }
    const VectorXs getLowerBound() const { return lowerBounds_; }
    const VectorXs getUpperBound() const { return upperBounds_; }
private:
    std::shared_ptr<ct::optcon::tpl::OptVector<SCALAR>> optVector_;
    Eigen::Matrix<SCALAR, KINEMATICS::NJOINTS, 1> contraints_;
    Eigen::Matrix<SCALAR, KINEMATICS::NJOINTS, 1> jacobian_;     // sparse jacobian
    Eigen::Matrix<SCALAR, KINEMATICS::NJOINTS, 1> lowerBounds_;  // lower bound
    Eigen::Matrix<SCALAR, KINEMATICS::NJOINTS, 1> upperBounds_;  // upper bound
};

}  // namespace rbd
}  // namespace ct
