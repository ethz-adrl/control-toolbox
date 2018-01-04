/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

//#include <ct/core/core.h>
#include <ct/core/internal/autodiff/SparsityPattern.h>

namespace ct {
namespace core {
namespace internal {

SparsityPattern::SparsityPattern(){};

SparsityPattern::~SparsityPattern(){};

const CppAD::vector<bool>& SparsityPattern::sparsity() const
{
    return sparsity_;
}

const CppAD::vector<size_t>& SparsityPattern::row() const
{
    return row_;
}

const CppAD::vector<size_t>& SparsityPattern::col() const
{
    return col_;
}

CppAD::sparse_jacobian_work& SparsityPattern::workJacobian()
{
    return workJacobian_;
}

CppAD::sparse_hessian_work& SparsityPattern::workHessian()
{
    return workHessian_;
}

void SparsityPattern::clearWork()
{
    workJacobian_.clear();
    workHessian_.clear();
}

} /* namespace internal */
} /* namespace core */
} /* namespace ct */
