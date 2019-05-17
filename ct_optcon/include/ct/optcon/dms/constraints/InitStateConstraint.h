/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/optcon/nlp/DiscreteConstraintBase.h>
#include <ct/optcon/dms/dms_core/OptVectorDms.h>

namespace ct {
namespace optcon {

/**
 * @ingroup    DMS
 *
 * @brief      The implementation of the DMS initial state constraint
 *
 * @tparam     STATE_DIM    The state dimension
 * @tparam     CONTROL_DIM  The input dimension
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class InitStateConstraint : public tpl::DiscreteConstraintBase<SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef DmsDimensions<STATE_DIM, CONTROL_DIM, SCALAR> DIMENSIONS;

    typedef tpl::DiscreteConstraintBase<SCALAR> BASE;
    typedef typename DIMENSIONS::state_vector_t state_vector_t;
    typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> VectorXs;

    /**
	 * @brief      Default constructor
	 */
    InitStateConstraint() = default;
    /**
	 * @brief      Custom constructor
	 *
	 * @param[in]  x0    The initial state
	 * @param[in]  w     The optimization variables
	 */
    InitStateConstraint(const state_vector_t& x0, std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>> w)
        : w_(w), x_0_(x0)
    {
        lb_.setConstant(SCALAR(0.0));
        ub_.setConstant(SCALAR(0.0));
    }

    /**
	 * @brief      Updates the constraint
	 *
	 * @param[in]  x0    The new initial state
	 */
    void updateConstraint(const state_vector_t& x0) { x_0_ = x0; }
    VectorXs eval() override { return w_->getOptimizedState(0) - x_0_; }
    VectorXs evalSparseJacobian() override { return state_vector_t::Ones(); }
    size_t getNumNonZerosJacobian() override { return (size_t)STATE_DIM; }
    void genSparsityPattern(Eigen::VectorXi& iRow_vec, Eigen::VectorXi& jCol_vec) override
    {
        size_t indexNumber = 0;
        indexNumber += BASE::genDiagonalIndices(w_->getStateIndex(0), STATE_DIM, iRow_vec, jCol_vec, indexNumber);
    }

    VectorXs getLowerBound() override { return lb_; }
    VectorXs getUpperBound() override { return ub_; }
    size_t getConstraintSize() override { return STATE_DIM; }
private:
    std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>> w_;
    state_vector_t x_0_;

    //Constraint bounds
    state_vector_t lb_;  // lower bound
    state_vector_t ub_;  // upper bound
};

}  // namespace optcon
}  // namespace ct
