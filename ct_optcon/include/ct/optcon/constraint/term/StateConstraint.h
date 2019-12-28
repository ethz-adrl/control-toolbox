/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {


/**
 * @ingroup    Constraint
 *
 * @brief      Class for state box constraint.
 *
 * @tparam     STATE_DIM  The state dimension
 * @tparam     INPUT_DIM  The control dimension
 * @tparam     SCALAR     The Scalar type
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class StateConstraint : public BoxConstraintBase<STATE_DIM, STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Trait = typename ct::core::tpl::TraitSelector<SCALAR>::Trait;
    using Base = BoxConstraintBase<STATE_DIM, STATE_DIM, CONTROL_DIM, SCALAR>;

    using state_vector_t = core::StateVector<STATE_DIM, SCALAR>;
    using control_vector_t = core::ControlVector<CONTROL_DIM, SCALAR>;

    using VectorXi = Eigen::Matrix<int, Eigen::Dynamic, 1>;
    using VectorXs = Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>;
    using MatrixXs = Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic>;

    using sparsity_matrix_t = Eigen::Matrix<SCALAR, Eigen::Dynamic, STATE_DIM>;

    /**
	 * @brief      Constructor taking lower and upper state bounds directly. Assumes state box constraint is dense.
	 *
	 * @param[in]  xLow   The lower state bound
	 * @param[in]  xHigh  The upper state bound
	 */
    StateConstraint(const state_vector_t& xLow, const state_vector_t& xHigh);

    /**
     * @brief 	  Constructor for sparse state box constraint. Takes bounds and sparsity pattern.
     * @param lb  Lower boundary values
     * @param ub  Upper boundary values
     * @param state_sparsity State constraint sparsity pattern
     */
    StateConstraint(const VectorXs& lb, const VectorXs& ub, const Eigen::VectorXi& state_sparsity);

    StateConstraint(const StateConstraint& arg);

    virtual ~StateConstraint();

    virtual StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override;

    virtual VectorXs evaluate(const state_vector_t& x, const control_vector_t& u, const SCALAR t) override;

#ifdef CPPADCG
    virtual Eigen::Matrix<ct::core::ADCGScalar, Eigen::Dynamic, 1> evaluateCppadCg(
        const core::StateVector<STATE_DIM, ct::core::ADCGScalar>& x,
        const core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar>& u,
        ct::core::ADCGScalar t) override;
#endif

    virtual MatrixXs jacobianState(const state_vector_t& x, const control_vector_t& u, const SCALAR t) override;

    virtual MatrixXs jacobianInput(const state_vector_t& x, const control_vector_t& u, const SCALAR t) override;

    virtual size_t getNumNonZerosJacobianState() const override;

    virtual size_t getNumNonZerosJacobianInput() const override;

    virtual VectorXs jacobianStateSparse(const state_vector_t& x, const control_vector_t& u, const SCALAR t) override;

    virtual VectorXs jacobianInputSparse(const state_vector_t& x, const control_vector_t& u, const SCALAR t) override;

    virtual void sparsityPatternState(VectorXi& rows, VectorXi& cols) override;

    virtual void sparsityPatternInput(VectorXi& rows, VectorXi& cols) override;
};

}  // namespace optcon
}  // namespace ct
