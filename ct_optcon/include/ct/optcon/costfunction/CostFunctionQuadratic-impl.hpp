/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <typename MANIFOLD, size_t CONTROL_DIM>
CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::CostFunctionQuadratic()
{
    eps_ = sqrt(Eigen::NumTraits<SCALAR_EVAL>::epsilon());
}

template <typename MANIFOLD, size_t CONTROL_DIM>
CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::CostFunctionQuadratic(const CostFunctionQuadratic& arg)
    : CostFunction<MANIFOLD, CONTROL_DIM>(arg), eps_(arg.eps_), doubleSidedDerivative_(arg.doubleSidedDerivative_)
{
    intermediateCostAnalytical_.resize(arg.intermediateCostAnalytical_.size());
    finalCostAnalytical_.resize(arg.finalCostAnalytical_.size());

    for (size_t i = 0; i < arg.intermediateCostAnalytical_.size(); i++)
    {
        intermediateCostAnalytical_[i] =
            std::shared_ptr<TermBase<MANIFOLD, CONTROL_DIM>>(arg.intermediateCostAnalytical_[i]->clone());
    }

    for (size_t i = 0; i < arg.finalCostAnalytical_.size(); i++)
    {
        finalCostAnalytical_[i] =
            std::shared_ptr<TermBase<MANIFOLD, CONTROL_DIM>>(arg.finalCostAnalytical_[i]->clone());
    }
}

template <typename MANIFOLD, size_t CONTROL_DIM>
CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::~CostFunctionQuadratic()
{
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::loadFromConfigFile(const std::string& filename, bool verbose)
{
    throw std::runtime_error("not implemented");
}

template <typename MANIFOLD, size_t CONTROL_DIM>
typename CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::control_vector_t
CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::controlDerivativeTerminal()
{
    throw std::runtime_error("controlDerivativeTerminal() not implemented in CostFunctionQuadratic");
}

template <typename MANIFOLD, size_t CONTROL_DIM>
typename CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::control_matrix_t
CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::controlSecondDerivativeTerminal()
{
    throw std::runtime_error("CostFunctionQuadratic: controlSecondDerivativeTerminal() not implemented");
}

template <typename MANIFOLD, size_t CONTROL_DIM>
typename CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::control_state_matrix_t
CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::stateControlDerivativeTerminal()
{
    throw std::runtime_error("stateControlDerivativeTerminal() not implemented");
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::updateReferenceState(const MANIFOLD& x_ref)
{
    for (auto costIntermediate : intermediateCostAnalytical_)
        costIntermediate->updateReferenceState(x_ref);
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::updateFinalState(const MANIFOLD& x_final)
{
    for (auto costFinal : finalCostAnalytical_)
        costFinal->updateReferenceState(x_final);
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::updateReferenceControl(const control_vector_t& u_ref)
{
    for (auto costIntermediate : intermediateCostAnalytical_)
        costIntermediate->updateReferenceControl(u_ref);
}

template <typename MANIFOLD, size_t CONTROL_DIM>
bool CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::stateDerivativeIntermediateTest(bool verbose)
{
    auto derivative = stateDerivativeIntermediate();
    auto derivativeNd = stateDerivativeIntermediateNumDiff();

    if (verbose)
        std::cout << "norm error between derivative/numdiff state : " << std::endl
                  << (derivative - derivativeNd).norm() << std::endl;

    return (derivative.isApprox(derivativeNd, 1e-6));
}

template <typename MANIFOLD, size_t CONTROL_DIM>
bool CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::controlDerivativeIntermediateTest(bool verbose)
{
    control_vector_t derivative = controlDerivativeIntermediate();
    control_vector_t derivativeNd = controlDerivativeIntermediateNumDiff();

    if (verbose)
        std::cout << "norm error between derivative/numdiff control : " << std::endl
                  << (derivative - derivativeNd).norm() << std::endl;

    return (derivative.isApprox(derivativeNd, 1e-6));
}

template <typename MANIFOLD, size_t CONTROL_DIM>
std::shared_ptr<TermBase<MANIFOLD, CONTROL_DIM>> CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::getIntermediateTermById(
    const size_t id)
{
    return intermediateCostAnalytical_[id];
}

template <typename MANIFOLD, size_t CONTROL_DIM>
std::shared_ptr<TermBase<MANIFOLD, CONTROL_DIM>> CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::getFinalTermById(
    const size_t id)
{
    return finalCostAnalytical_[id];
}

template <typename MANIFOLD, size_t CONTROL_DIM>
std::shared_ptr<TermBase<MANIFOLD, CONTROL_DIM>>
CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::getIntermediateTermByName(const std::string& name)
{
    for (auto term : intermediateCostAnalytical_)
        if (term->getName() == name)
            return term;

    throw std::runtime_error("Term " + name + " not found in the CostFunctionQuadratic");
}

template <typename MANIFOLD, size_t CONTROL_DIM>
std::shared_ptr<TermBase<MANIFOLD, CONTROL_DIM>> CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::getFinalTermByName(
    const std::string& name)
{
    for (auto term : finalCostAnalytical_)
        if (term->getName() == name)
            return term;

    throw std::runtime_error("Term " + name + " not found in the CostFunctionQuadratic");
}


template <typename MANIFOLD, size_t CONTROL_DIM>
void CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::initialize()
{
    /*!
	 * do nothing at all
	 */
}


// add terms
template <typename MANIFOLD, size_t CONTROL_DIM>
size_t CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::addIntermediateTerm(
    std::shared_ptr<TermBase<MANIFOLD, CONTROL_DIM>> term,
    bool verbose)
{
    intermediateCostAnalytical_.push_back(term);
    if (verbose)
    {
        std::string name = term->getName();
        std::cout << "Trying to add term as intermediate" << std::endl;
    }

    return intermediateCostAnalytical_.size() - 1;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
size_t CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::addFinalTerm(std::shared_ptr<TermBase<MANIFOLD, CONTROL_DIM>> term,
    bool verbose)
{
    finalCostAnalytical_.push_back(term);
    if (verbose)
    {
        std::string name = term->getName();
        std::cout << "Trying to add term as final" << std::endl;
    }

    return finalCostAnalytical_.size() - 1;
}


template <typename MANIFOLD, size_t CONTROL_DIM>
auto CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::evaluateIntermediateBase() -> SCALAR_EVAL
{
    SCALAR_EVAL y = SCALAR_EVAL(0.0);

    for (auto it : this->intermediateCostAnalytical_)
    {
        if (!it->isActiveAtTime(this->t_))
        {
            continue;
        }
        y += it->computeActivation(this->t_) * it->evaluate(this->x_, this->u_, this->t_);
    }

    return y;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
auto CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::evaluateTerminalBase() -> SCALAR_EVAL
{
    SCALAR_EVAL y = SCALAR_EVAL(0.0);

    for (auto it : this->finalCostAnalytical_)
        y += it->evaluate(this->x_, this->u_, this->t_);

    return y;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
auto CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::stateDerivativeIntermediateBase() -> typename EVAL_MANIFOLD::Tangent
{
    typename EVAL_MANIFOLD::Tangent derivative;
    derivative.setZero();

    for (auto it : this->intermediateCostAnalytical_)
    {
        if (!it->isActiveAtTime(this->t_))
        {
            continue;
        }
        derivative += it->computeActivation(this->t_) * it->stateDerivative(this->x_, this->u_, this->t_);
    }

    return derivative;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
auto CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::stateDerivativeTerminalBase() -> typename EVAL_MANIFOLD::Tangent
{
    typename EVAL_MANIFOLD::Tangent derivative;
    derivative.setZero();

    for (auto it : this->finalCostAnalytical_)
        derivative += it->stateDerivative(this->x_, this->u_, this->t_);

    return derivative;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
typename CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::state_matrix_t
CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::stateSecondDerivativeIntermediateBase()
{
    state_matrix_t derivative;
    derivative.setZero();

    for (auto it : this->intermediateCostAnalytical_)
    {
        if (!it->isActiveAtTime(this->t_))
        {
            continue;
        }
        derivative += it->computeActivation(this->t_) * it->stateSecondDerivative(this->x_, this->u_, this->t_);
    }

    return derivative;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
typename CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::state_matrix_t
CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::stateSecondDerivativeTerminalBase()
{
    state_matrix_t derivative;
    derivative.setZero();

    for (auto it : this->finalCostAnalytical_)
        derivative += it->stateSecondDerivative(this->x_, this->u_, this->t_);

    return derivative;
}


template <typename MANIFOLD, size_t CONTROL_DIM>
typename CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::control_vector_t
CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::controlDerivativeIntermediateBase()
{
    control_vector_t derivative;
    derivative.setZero();

    for (auto it : this->intermediateCostAnalytical_)
    {
        if (!it->isActiveAtTime(this->t_))
        {
            continue;
        }
        derivative += it->computeActivation(this->t_) * it->controlDerivative(this->x_, this->u_, this->t_);
    }

    return derivative;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
typename CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::control_vector_t
CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::controlDerivativeTerminalBase()
{
    control_vector_t derivative;
    derivative.setZero();

    for (auto it : this->finalCostAnalytical_)
        derivative += it->controlDerivative(this->x_, this->u_, this->t_);

    return derivative;
}

// get control second derivatives
template <typename MANIFOLD, size_t CONTROL_DIM>
typename CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::control_matrix_t
CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::controlSecondDerivativeIntermediateBase()
{
    control_matrix_t derivative;
    derivative.setZero();

    for (auto it : this->intermediateCostAnalytical_)
    {
        if (!it->isActiveAtTime(this->t_))
        {
            continue;
        }
        derivative += it->computeActivation(this->t_) * it->controlSecondDerivative(this->x_, this->u_, this->t_);
    }

    return derivative;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
typename CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::control_matrix_t
CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::controlSecondDerivativeTerminalBase()
{
    control_matrix_t derivative;
    derivative.setZero();

    for (auto it : this->finalCostAnalytical_)
        derivative += it->controlSecondDerivative(this->x_, this->u_, this->t_);

    return derivative;
}

// get state-control derivatives
template <typename MANIFOLD, size_t CONTROL_DIM>
typename CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::control_state_matrix_t
CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::stateControlDerivativeIntermediateBase()
{
    control_state_matrix_t derivative;
    derivative.setZero();

    for (auto it : this->intermediateCostAnalytical_)
    {
        if (!it->isActiveAtTime(this->t_))
        {
            continue;
        }
        derivative += it->computeActivation(this->t_) * it->stateControlDerivative(this->x_, this->u_, this->t_);
    }

    return derivative;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
typename CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::control_state_matrix_t
CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>::stateControlDerivativeTerminalBase()
{
    control_state_matrix_t derivative;
    derivative.setZero();

    for (auto it : this->finalCostAnalytical_)
        derivative += it->stateControlDerivative(this->x_, this->u_, this->t_);

    return derivative;
}

}  // namespace optcon
}  // namespace ct
