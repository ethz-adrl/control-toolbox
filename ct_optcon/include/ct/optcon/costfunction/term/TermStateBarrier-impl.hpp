/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermStateBarrier<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::TermStateBarrier(const state_vector_t& ub,
    const state_vector_t& lb,
    const state_vector_t& alpha)
    : ub_(ub), lb_(lb), alpha_(alpha)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermStateBarrier<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::TermStateBarrier()
    : ub_(state_vector_t::Zero()), lb_(state_vector_t::Zero()), alpha_(state_vector_t::Zero())
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermStateBarrier<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::TermStateBarrier(const TermStateBarrier& arg)
    : ub_(arg.ub_), lb_(arg.lb_), alpha_(arg.alpha_)
{
    initialize();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermStateBarrier<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::~TermStateBarrier()
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TermStateBarrier<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>*
TermStateBarrier<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::clone() const
{
    return new TermStateBarrier(*this);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
void TermStateBarrier<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::initialize()
{
    barriers_.clear();
    for (size_t i = 0; i < STATE_DIM; i++)
    {
        barriers_.push_back(ct::core::tpl::BarrierActivation<SCALAR>(ub_(i), lb_(i), alpha_(i)));
    }
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
SCALAR TermStateBarrier<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::evaluate(
    const Eigen::Matrix<SCALAR, STATE_DIM, 1>& x,
    const Eigen::Matrix<SCALAR, CONTROL_DIM, 1>& u,
    const SCALAR& t)
{
    SCALAR c = SCALAR(0.0);
    for (size_t i = 0; i < STATE_DIM; i++)
        c += barriers_[i].computeActivation(x(i));
    return c;
}

#ifdef CPPADCG
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
ct::core::ADCGScalar TermStateBarrier<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::evaluateCppadCg(
    const core::StateVector<STATE_DIM, ct::core::ADCGScalar>& x,
    const core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar>& u,
    ct::core::ADCGScalar t)
{
    ct::core::ADCGScalar c = ct::core::ADCGScalar(0.0);
    for (size_t i = 0; i < STATE_DIM; i++)
        c += barriers_[i].computeActivation(x(i));
    return c;
}
#endif

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
void TermStateBarrier<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::loadConfigFile(const std::string& filename,
    const std::string& termName,
    bool verbose)
{
    state_matrix_t Alpha;
    state_matrix_t Ub;
    state_matrix_t Lb;

    loadMatrixCF(filename, "alpha", Alpha, termName);
    loadMatrixCF(filename, "upper_bound", Ub, termName);
    loadMatrixCF(filename, "lower_bound", Lb, termName);

    alpha_ = Alpha.diagonal();
    ub_ = Ub.diagonal();
    lb_ = Lb.diagonal();

    if (verbose)
    {
        std::cout << "Read alpha as = \n" << alpha_.transpose() << std::endl;
        std::cout << "Read upper_bound as = \n" << ub_.transpose() << std::endl;
        std::cout << "Read lower_bound as = \n" << lb_.transpose() << std::endl;
    }
}
}  // namespace optcon
}  // namespace ct
