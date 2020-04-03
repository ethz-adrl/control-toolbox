/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <typename MANIFOLD, size_t CONTROL_DIM>
CostFunctionAnalytical<MANIFOLD, CONTROL_DIM>::CostFunctionAnalytical()
{
}

template <typename MANIFOLD, size_t CONTROL_DIM>
CostFunctionAnalytical<MANIFOLD, CONTROL_DIM>::CostFunctionAnalytical(const CostFunctionAnalytical& arg)
    : CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>(arg)
{
}

template <typename MANIFOLD, size_t CONTROL_DIM>
CostFunctionAnalytical<MANIFOLD, CONTROL_DIM>::CostFunctionAnalytical(const std::string& filename, bool verbose)
{
    loadFromConfigFile(filename, verbose);
}

template <typename MANIFOLD, size_t CONTROL_DIM>
CostFunctionAnalytical<MANIFOLD, CONTROL_DIM>* CostFunctionAnalytical<MANIFOLD, CONTROL_DIM>::clone() const
{
    return new CostFunctionAnalytical(*this);
}

template <typename MANIFOLD, size_t CONTROL_DIM>
CostFunctionAnalytical<MANIFOLD, CONTROL_DIM>::~CostFunctionAnalytical()
{
}


template <typename MANIFOLD, size_t CONTROL_DIM>
void CostFunctionAnalytical<MANIFOLD, CONTROL_DIM>::loadFromConfigFile(const std::string& filename, bool verbose)
{
    if (verbose)
        std::cout << "Starting to load analytical cost function from file " << filename << std::endl;

    this->intermediateCostAnalytical_.clear();
    this->finalCostAnalytical_.clear();

    boost::property_tree::ptree pt;
    boost::property_tree::read_info(filename, pt);
    int i = 0;
    std::string currentTerm;
    do
    {
        std::cout << "=============================================" << std::endl;  //indicating new term
        currentTerm = "term" + std::to_string(i);
        std::string termKind = pt.get<std::string>(currentTerm + ".kind");
        boost::algorithm::to_lower(termKind);
        int currentTermType = pt.get<int>(currentTerm + ".type");
        std::string termName;
        try
        {
            termName = pt.get<std::string>(currentTerm + ".name");
            if (verbose)
                std::cout << "Trying to add " + termName + " as term" << std::endl;
        } catch (boost::property_tree::ptree_bad_path err)
        {
            termName = "Unnamed";
            if (verbose)
            {
                std::cout << "Name field for " + currentTerm + " does not exist" << std::endl;
            }
        }

        std::shared_ptr<TermBase<MANIFOLD, CONTROL_DIM>> term;

        CT_LOADABLE_TERMS(MANIFOLD, CONTROL_DIM);

        if (!term)
        {
            throw std::runtime_error("Term type \"" + termKind + "\" not supported");
        }
        else
        {
            addTerm(filename, currentTerm, currentTermType, term, this, verbose);
        }
        currentTerm = "term" + std::to_string(++i);
    } while (pt.find(currentTerm) != pt.not_found());
}

template <typename MANIFOLD, size_t CONTROL_DIM>
auto CostFunctionAnalytical<MANIFOLD, CONTROL_DIM>::evaluateIntermediate() -> SCALAR_EVAL
{
    return this->evaluateIntermediateBase();
}

template <typename MANIFOLD, size_t CONTROL_DIM>
auto CostFunctionAnalytical<MANIFOLD, CONTROL_DIM>::evaluateTerminal() -> SCALAR_EVAL
{
    return this->evaluateTerminalBase();
}

template <typename MANIFOLD, size_t CONTROL_DIM>
auto CostFunctionAnalytical<MANIFOLD, CONTROL_DIM>::stateDerivativeIntermediate()
    -> ct::core::StateVector<STATE_DIM, SCALAR_EVAL>
{
    return this->stateDerivativeIntermediateBase();
}

template <typename MANIFOLD, size_t CONTROL_DIM>
auto CostFunctionAnalytical<MANIFOLD, CONTROL_DIM>::stateDerivativeTerminal()
    -> ct::core::StateVector<STATE_DIM, SCALAR_EVAL>
{
    return this->stateDerivativeTerminalBase();
}

template <typename MANIFOLD, size_t CONTROL_DIM>
typename CostFunctionAnalytical<MANIFOLD, CONTROL_DIM>::state_matrix_t
CostFunctionAnalytical<MANIFOLD, CONTROL_DIM>::stateSecondDerivativeIntermediate()
{
    return this->stateSecondDerivativeIntermediateBase();
}

template <typename MANIFOLD, size_t CONTROL_DIM>
typename CostFunctionAnalytical<MANIFOLD, CONTROL_DIM>::state_matrix_t
CostFunctionAnalytical<MANIFOLD, CONTROL_DIM>::stateSecondDerivativeTerminal()
{
    return this->stateSecondDerivativeTerminalBase();
}

template <typename MANIFOLD, size_t CONTROL_DIM>
typename CostFunctionAnalytical<MANIFOLD, CONTROL_DIM>::control_vector_t
CostFunctionAnalytical<MANIFOLD, CONTROL_DIM>::controlDerivativeIntermediate()
{
    return this->controlDerivativeIntermediateBase();
}

template <typename MANIFOLD, size_t CONTROL_DIM>
typename CostFunctionAnalytical<MANIFOLD, CONTROL_DIM>::control_vector_t
CostFunctionAnalytical<MANIFOLD, CONTROL_DIM>::controlDerivativeTerminal()
{
    return this->controlDerivativeTerminalBase();
}

template <typename MANIFOLD, size_t CONTROL_DIM>
typename CostFunctionAnalytical<MANIFOLD, CONTROL_DIM>::control_matrix_t
CostFunctionAnalytical<MANIFOLD, CONTROL_DIM>::controlSecondDerivativeIntermediate()
{
    return this->controlSecondDerivativeIntermediateBase();
}

template <typename MANIFOLD, size_t CONTROL_DIM>
typename CostFunctionAnalytical<MANIFOLD, CONTROL_DIM>::control_matrix_t
CostFunctionAnalytical<MANIFOLD, CONTROL_DIM>::controlSecondDerivativeTerminal()
{
    return this->controlSecondDerivativeTerminalBase();
}

template <typename MANIFOLD, size_t CONTROL_DIM>
typename CostFunctionAnalytical<MANIFOLD, CONTROL_DIM>::control_state_matrix_t
CostFunctionAnalytical<MANIFOLD, CONTROL_DIM>::stateControlDerivativeIntermediate()
{
    return this->stateControlDerivativeIntermediateBase();
}

template <typename MANIFOLD, size_t CONTROL_DIM>
typename CostFunctionAnalytical<MANIFOLD, CONTROL_DIM>::control_state_matrix_t
CostFunctionAnalytical<MANIFOLD, CONTROL_DIM>::stateControlDerivativeTerminal()
{
    return this->stateControlDerivativeTerminalBase();
}

}  // namespace optcon
}  // namespace ct
