/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

/**
 * @brief      Base for box constraint, templated on dimension of the decision vector of the derived class
 *
 * @tparam     DERIVED_DIM Dimension of the decision vector of the derived class
 * @tparam     STATE_DIM  The state dimension
 * @tparam     INPUT_DIM  The control dimension
 * @tparam     SCALAR     The Scalar type
 */
template <size_t DERIVED_DIM, size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class BoxConstraintBase : public ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Trait = typename ct::core::tpl::TraitSelector<SCALAR>::Trait;
    using Base = ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>;

    using state_vector_t = core::StateVector<STATE_DIM, SCALAR>;
    using control_vector_t = core::ControlVector<CONTROL_DIM, SCALAR>;
    using decision_vector_t = core::StateVector<DERIVED_DIM, SCALAR>;

    using VectorXi = Eigen::Matrix<int, Eigen::Dynamic, 1>;
    using VectorXs = Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>;
    using MatrixXs = Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic>;

    using sparsity_matrix_t = Eigen::Matrix<SCALAR, Eigen::Dynamic, DERIVED_DIM>;

    //! generate sparsity pattern for sparse box constraint
    static void sparsityPatternSparseJacobian(const VectorXi& sparsity_vec,
        const size_t& constrSize,
        VectorXi& rows,
        VectorXi& cols);

    /**
	 * @brief      Constructor taking lower and upper state bounds directly. Assumes the box constraint is dense.
	 *
	 * @param[in]  vLow   The full lower bound
	 * @param[in]  vHigh  The full upper bound
	 */
    BoxConstraintBase(const decision_vector_t& vLow, const decision_vector_t& vHigh);

    /**
     * @brief 	  Constructor for sparse box constraint. Takes bounds and sparsity pattern.
     * @param lb  Lower boundary values
     * @param ub  Upper boundary values
     * @param sparsity_vec Box constraint sparsity pattern as a vector
     */
    BoxConstraintBase(const VectorXs& lb, const VectorXs& ub, const Eigen::VectorXi& sparsity_vec);

    BoxConstraintBase(const BoxConstraintBase& arg);

    virtual ~BoxConstraintBase();

    virtual BoxConstraintBase<DERIVED_DIM, STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override = 0;

    virtual size_t getConstraintSize() const override;

    virtual VectorXs evaluate(const state_vector_t& x, const control_vector_t& u, const SCALAR t) override = 0;

#ifdef CPPADCG
    virtual Eigen::Matrix<ct::core::ADCGScalar, Eigen::Dynamic, 1> evaluateCppadCg(
        const core::StateVector<STATE_DIM, ct::core::ADCGScalar>& x,
        const core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar>& u,
        ct::core::ADCGScalar t) override = 0;
#endif

    virtual MatrixXs jacobianState(const state_vector_t& x, const control_vector_t& u, const SCALAR t) override = 0;

    virtual MatrixXs jacobianInput(const state_vector_t& x, const control_vector_t& u, const SCALAR t) override = 0;

    virtual size_t getNumNonZerosJacobianState() const override = 0;

    virtual size_t getNumNonZerosJacobianInput() const override = 0;

    virtual VectorXs jacobianStateSparse(const state_vector_t& x,
        const control_vector_t& u,
        const SCALAR t) override = 0;

    virtual VectorXs jacobianInputSparse(const state_vector_t& x,
        const control_vector_t& u,
        const SCALAR t) override = 0;

    virtual void sparsityPatternState(VectorXi& rows, VectorXi& cols) override = 0;

    virtual void sparsityPatternInput(VectorXi& rows, VectorXi& cols) override = 0;

protected:
    /*!
     * @brief transform a sparsity vector (giving the sparsity pattern on the diagonal) in to a sparsity matrix
     * @param spVec diagonal sparsity pattern, e.g. [0 0 1 0 1 0]
     * @param nConstr number of constraints
     * @return the sparsity matrix
     */
    sparsity_matrix_t diagSparsityVecToSparsityMat(const VectorXi& spVec, const size_t& nConstr);

    //! sparsity in vector form
    VectorXi sparsity_;

    //! sparsity matrix
    sparsity_matrix_t sparsity_J_;

    //! size of the constraint
    size_t constrSize_;

private:
    void sanityCheck(const size_t& nCon, const VectorXs& lb, const VectorXs& ub) const;
};

}  // namespace optcon
}  // namepace ct

/*
 * For this class, we can include the implementation here, as it is virtual,
 * and only used by few derived members who all get compiled in prespec themselves.
 */
#include "BoxConstraintBase-impl.h"
