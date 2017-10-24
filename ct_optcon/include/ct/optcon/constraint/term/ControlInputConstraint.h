/**********************************************************************************************************************
This file is part of the Control Toobox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Lincensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

/**
 * @ingroup    Constraint
 *
 * @brief      Class for control input constraint.
 *
 * @tparam     STATE_DIM    The state dimension
 * @tparam     CONTROL_DIM  The control dimension
 * @tparam     SCALAR       The Scalar type
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class ControlInputConstraint : public ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef typename ct::core::tpl::TraitSelector<SCALAR>::Trait Trait;
    typedef ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR> Base;

    typedef core::StateVector<STATE_DIM, SCALAR> state_vector_t;
    typedef core::ControlVector<CONTROL_DIM, SCALAR> control_vector_t;

    typedef Eigen::Matrix<int, Eigen::Dynamic, 1> VectorXi;
    typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> VectorXs;
    typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic> MatrixXs;

    /**
	 * @brief      Custom constructor
	 *
	 * @param[in]  uLow   The upper control input bound
	 * @param[in]  uHigh  The lower control input bound
	 */
    ControlInputConstraint(const core::ControlVector<CONTROL_DIM> uLow, const core::ControlVector<CONTROL_DIM> uHigh);

    virtual ~ControlInputConstraint();

    virtual ControlInputConstraint<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override;

    virtual size_t getConstraintSize() const override;

    virtual Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> evaluate(const state_vector_t& x,
        const control_vector_t& u,
        const SCALAR t) override;

    virtual Eigen::Matrix<ct::core::ADCGScalar, Eigen::Dynamic, 1> evaluateCppadCg(
        const core::StateVector<STATE_DIM, ct::core::ADCGScalar>& x,
        const core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar>& u,
        ct::core::ADCGScalar t) override;

    virtual MatrixXs jacobianState(const state_vector_t& x, const control_vector_t& u, const SCALAR t) override;

    virtual MatrixXs jacobianInput(const state_vector_t& x, const control_vector_t& u, const SCALAR t) override;

    virtual size_t getNumNonZerosJacobianState() const override;

    virtual size_t getNumNonZerosJacobianInput() const override;

    virtual VectorXs jacobianInputSparse(const state_vector_t& x, const control_vector_t& u, const SCALAR t) override;

    virtual void sparsityPatternInput(VectorXi& rows, VectorXi& cols) override;
};
}
}
