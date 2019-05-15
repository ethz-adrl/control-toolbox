/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

/**
 * @brief      Class for obstacle constraint.
 *
 * @tparam     STATE_DIM  The state dimension
 * @tparam     INPUT_DIM  The control dimension
 * @tparam     SCALAR     The Scalar type
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class ObstacleConstraint : public ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef typename ct::core::tpl::TraitSelector<SCALAR>::Trait Trait;
    typedef ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR> Base;

    typedef core::StateVector<STATE_DIM, SCALAR> state_vector_t;
    typedef core::ControlVector<CONTROL_DIM, SCALAR> control_vector_t;

    typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> VectorXs;
    typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic> MatrixXs;

    typedef Eigen::Matrix<SCALAR, 3, 1> Vector3s;

    ObstacleConstraint(std::shared_ptr<ct::core::tpl::Ellipsoid<SCALAR>> obstacle,
        std::function<void(const state_vector_t&, Vector3s&)> getPosition,
        std::function<void(const state_vector_t&, Eigen::Matrix<SCALAR, 3, STATE_DIM>&)> getJacobian);

    virtual ObstacleConstraint<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override;

    virtual ~ObstacleConstraint();

    ObstacleConstraint(const ObstacleConstraint& arg);

    virtual size_t getConstraintSize() const override;

    virtual Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> evaluate(const state_vector_t& x,
        const control_vector_t& u,
        const SCALAR t) override;

    virtual MatrixXs jacobianState(const state_vector_t& x, const control_vector_t& u, const SCALAR t) override;

    virtual MatrixXs jacobianInput(const state_vector_t& x, const control_vector_t& u, const SCALAR t) override;

private:
    std::shared_ptr<ct::core::tpl::Ellipsoid<SCALAR>> obstacle_;

    std::function<void(const core::StateVector<STATE_DIM, SCALAR>&, Vector3s&)> xFun_;
    std::function<void(const core::StateVector<STATE_DIM, SCALAR>&, Eigen::Matrix<SCALAR, 3, STATE_DIM>&)> dXFun_;

    core::StateVector<1, SCALAR> val_;
    Eigen::Matrix<SCALAR, 1, STATE_DIM> jac_;
};

}  // namespace optcon
}  // namespace ct
