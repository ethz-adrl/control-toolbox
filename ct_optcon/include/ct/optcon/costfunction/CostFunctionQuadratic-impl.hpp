/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::CostFunctionQuadratic()
{
    eps_ = sqrt(Eigen::NumTraits<SCALAR>::epsilon());
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::CostFunctionQuadratic(const CostFunctionQuadratic& arg)
    : CostFunction<STATE_DIM, CONTROL_DIM, SCALAR>(arg),
      eps_(arg.eps_),
      doubleSidedDerivative_(arg.doubleSidedDerivative_)
{
    intermediateCostAnalytical_.resize(arg.intermediateCostAnalytical_.size());
    finalCostAnalytical_.resize(arg.finalCostAnalytical_.size());

    for (size_t i = 0; i < arg.intermediateCostAnalytical_.size(); i++)
    {
        intermediateCostAnalytical_[i] =
            std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR>>(arg.intermediateCostAnalytical_[i]->clone());
    }

    for (size_t i = 0; i < arg.finalCostAnalytical_.size(); i++)
    {
        finalCostAnalytical_[i] =
            std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR>>(arg.finalCostAnalytical_[i]->clone());
    }
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::~CostFunctionQuadratic()
{
}

#ifdef CPPADCG
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::addIntermediateADTerm(
    std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR, ct::core::ADCGScalar>> term,
    bool verbose)
{
    throw std::runtime_error("not implemented");
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::addFinalADTerm(
    std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR, ct::core::ADCGScalar>> term,
    bool verbose)
{
    throw std::runtime_error("not implemented");
}
#endif

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::loadFromConfigFile(const std::string& filename,
    bool verbose)
{
    throw std::runtime_error("not implemented");
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::control_vector_t
CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::controlDerivativeTerminal()
{
    throw std::runtime_error("controlDerivativeTerminal() not implemented in CostFunctionQuadratic");
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::control_matrix_t
CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::controlSecondDerivativeTerminal()
{
    throw std::runtime_error("controlSecondDerivativeTerminal() not implemented");
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::control_state_matrix_t
CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::stateControlDerivativeTerminal()
{
    throw std::runtime_error("stateControlDerivativeTerminal() not implemented");
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::updateReferenceState(const state_vector_t& x_ref)
{
    for (auto costIntermediate : intermediateCostAnalytical_)
        costIntermediate->updateReferenceState(x_ref);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::updateFinalState(const state_vector_t& x_final)
{
    for (auto costFinal : finalCostAnalytical_)
        costFinal->updateReferenceState(x_final);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::updateReferenceControl(const control_vector_t& u_ref)
{
    for (auto costIntermediate : intermediateCostAnalytical_)
        costIntermediate->updateReferenceControl(u_ref);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
bool CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::stateDerivativeIntermediateTest(bool verbose)
{
    state_vector_t derivative = stateDerivativeIntermediate();
    state_vector_t derivativeNd = stateDerivativeIntermediateNumDiff();

    if (verbose)
        std::cout << "norm error between derivative/numdiff state : " << std::endl
                  << (derivative - derivativeNd).norm() << std::endl;

    return (derivative.isApprox(derivativeNd, 1e-6));
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
bool CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::controlDerivativeIntermediateTest(bool verbose)
{
    control_vector_t derivative = controlDerivativeIntermediate();
    control_vector_t derivativeNd = controlDerivativeIntermediateNumDiff();

    if (verbose)
        std::cout << "norm error between derivative/numdiff control : " << std::endl
                  << (derivative - derivativeNd).norm() << std::endl;

    return (derivative.isApprox(derivativeNd, 1e-6));
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR>>
CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::getIntermediateTermById(const size_t id)
{
    return intermediateCostAnalytical_[id];
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR>>
CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::getFinalTermById(const size_t id)
{
    return finalCostAnalytical_[id];
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR>>
CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::getIntermediateTermByName(const std::string& name)
{
    for (auto term : intermediateCostAnalytical_)
        if (term->getName() == name)
            return term;

    throw std::runtime_error("Term " + name + " not found in the CostFunctionQuadratic");
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR>>
CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::getFinalTermByName(const std::string& name)
{
    for (auto term : finalCostAnalytical_)
        if (term->getName() == name)
            return term;

    throw std::runtime_error("Term " + name + " not found in the CostFunctionQuadratic");
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::state_vector_t
CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::stateDerivativeIntermediateNumDiff()
{
    state_vector_t dFdx = state_vector_t::Zero();
    state_vector_t x_local;
    control_vector_t u_local;
    SCALAR t_local;
    this->getCurrentStateAndControl(x_local, u_local, t_local);
    SCALAR dxdt_ref = this->evaluateIntermediate();

    for (size_t i = 0; i < STATE_DIM; ++i)
    {
        // inspired from http://en.wikipedia.org/wiki/Numerical_differentiation#Practical_considerations_using_floating_point_arithmetic
        SCALAR h = eps_ * std::max(std::abs(x_local(i)), 1.0);
        volatile SCALAR x_ph = x_local(i) + h;
        SCALAR dxp = x_ph - x_local(i);

        state_vector_t x_perturbed = x_local;
        x_perturbed(i) = x_ph;

        // get evaluation of f(x,u)
        this->setCurrentStateAndControl(x_perturbed, u_local, t_local);
        SCALAR dxdt = this->evaluateIntermediate();

        if (doubleSidedDerivative_)
        {
            SCALAR dxdt_low;

            volatile SCALAR x_mh = x_local(i) - h;
            SCALAR dxm = x_local(i) - x_mh;

            x_perturbed = x_local;
            x_perturbed(i) = x_mh;
            this->setCurrentStateAndControl(x_perturbed, u_local, t_local);
            dxdt_low = this->evaluateIntermediate();
            dFdx(i, 0) = (dxdt - dxdt_low) / (dxp + dxm);
        }
        else
        {
            dFdx(i, 0) = (dxdt - dxdt_ref) / dxp;
        }
    }

    return dFdx;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::control_vector_t
CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::controlDerivativeIntermediateNumDiff()
{
    control_vector_t dFdu = control_vector_t::Zero();
    state_vector_t x_local;
    control_vector_t u_local;
    SCALAR t_local;
    this->getCurrentStateAndControl(x_local, u_local, t_local);
    SCALAR dxdt_ref = this->evaluateIntermediate();

    for (size_t i = 0; i < CONTROL_DIM; ++i)
    {
        // inspired from http://en.wikipedia.org/wiki/Numerical_differentiation#Practical_considerations_using_floating_point_arithmetic
        SCALAR h = eps_ * std::max(std::abs(u_local(i)), 1.0);
        volatile SCALAR u_ph = u_local(i) + h;
        SCALAR dup = u_ph - u_local(i);

        control_vector_t u_perturbed = u_local;
        u_perturbed(i) = u_ph;

        // get evaluation of f(x,u)
        this->setCurrentStateAndControl(x_local, u_perturbed, t_local);
        SCALAR dxdt = this->evaluateIntermediate();

        if (doubleSidedDerivative_)
        {
            SCALAR dxdt_low;

            volatile SCALAR u_mh = u_local(i) - h;
            SCALAR dum = u_local(i) - u_mh;

            u_perturbed = u_local;
            u_perturbed(i) = u_mh;
            this->setCurrentStateAndControl(x_local, u_perturbed, t_local);
            dxdt_low = this->evaluateIntermediate();

            dFdu(i, 0) = (dxdt - dxdt_low) / (dup + dum);
        }
        else
        {
            dFdu(i, 0) = (dxdt - dxdt_ref) / dup;
        }
    }

    return dFdu;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::initialize()
{
    /*!
	 * do nothing at all
	 */
}


// add terms
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::addIntermediateTerm(
    std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR>> term,
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

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::addFinalTerm(
    std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR>> term,
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


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
SCALAR CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::evaluateIntermediateBase()
{
    SCALAR y = SCALAR(0.0);

    for (auto it : this->intermediateCostAnalytical_)
    {
        if (!it->isActiveAtTime(this->t_))
        {
            continue;
        }
        y += it->computeActivation(this->t_) * it->eval(this->x_, this->u_, this->t_);
    }

    return y;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
SCALAR CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::evaluateTerminalBase()
{
    SCALAR y = SCALAR(0.0);

    for (auto it : this->finalCostAnalytical_)
        y += it->evaluate(this->x_, this->u_, this->t_);

    return y;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::state_vector_t
CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::stateDerivativeIntermediateBase()
{
    state_vector_t derivative;
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


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::state_vector_t
CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::stateDerivativeTerminalBase()
{
    state_vector_t derivative;
    derivative.setZero();

    for (auto it : this->finalCostAnalytical_)
        derivative += it->stateDerivative(this->x_, this->u_, this->t_);

    return derivative;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::state_matrix_t
CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::stateSecondDerivativeIntermediateBase()
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

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::state_matrix_t
CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::stateSecondDerivativeTerminalBase()
{
    state_matrix_t derivative;
    derivative.setZero();

    for (auto it : this->finalCostAnalytical_)
        derivative += it->stateSecondDerivative(this->x_, this->u_, this->t_);

    return derivative;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::control_vector_t
CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::controlDerivativeIntermediateBase()
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

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::control_vector_t
CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::controlDerivativeTerminalBase()
{
    control_vector_t derivative;
    derivative.setZero();

    for (auto it : this->finalCostAnalytical_)
        derivative += it->controlDerivative(this->x_, this->u_, this->t_);

    return derivative;
}

// get control second derivatives
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::control_matrix_t
CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::controlSecondDerivativeIntermediateBase()
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

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::control_matrix_t
CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::controlSecondDerivativeTerminalBase()
{
    control_matrix_t derivative;
    derivative.setZero();

    for (auto it : this->finalCostAnalytical_)
        derivative += it->controlSecondDerivative(this->x_, this->u_, this->t_);

    return derivative;
}

// get state-control derivatives
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::control_state_matrix_t
CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::stateControlDerivativeIntermediateBase()
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

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::control_state_matrix_t
CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::stateControlDerivativeTerminalBase()
{
    control_state_matrix_t derivative;
    derivative.setZero();

    for (auto it : this->finalCostAnalytical_)
        derivative += it->stateControlDerivative(this->x_, this->u_, this->t_);

    return derivative;
}

}  // namespace optcon
}  // namespace ct
