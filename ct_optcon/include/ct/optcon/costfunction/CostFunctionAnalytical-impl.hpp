/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::CostFunctionAnalytical()
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::CostFunctionAnalytical(const CostFunctionAnalytical& arg)
    : CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>(arg)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::CostFunctionAnalytical(const std::string& filename,
    bool verbose)
{
    loadFromConfigFile(filename, verbose);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>* CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::clone()
    const
{
    return new CostFunctionAnalytical(*this);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::~CostFunctionAnalytical()
{
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::loadFromConfigFile(const std::string& filename,
    bool verbose)
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

        std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR>> term;

        CT_LOADABLE_TERMS(SCALAR, SCALAR);

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

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
SCALAR CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::evaluateIntermediate()
{
    return this->evaluateIntermediateBase();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
SCALAR CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::evaluateTerminal()
{
    return this->evaluateTerminalBase();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::state_vector_t
CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::stateDerivativeIntermediate()
{
    return this->stateDerivativeIntermediateBase();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::state_vector_t
CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::stateDerivativeTerminal()
{
    return this->stateDerivativeTerminalBase();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::state_matrix_t
CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::stateSecondDerivativeIntermediate()
{
    return this->stateSecondDerivativeIntermediateBase();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::state_matrix_t
CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::stateSecondDerivativeTerminal()
{
    return this->stateSecondDerivativeTerminalBase();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::control_vector_t
CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::controlDerivativeIntermediate()
{
    return this->controlDerivativeIntermediateBase();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::control_vector_t
CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::controlDerivativeTerminal()
{
    return this->controlDerivativeTerminalBase();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::control_matrix_t
CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::controlSecondDerivativeIntermediate()
{
    return this->controlSecondDerivativeIntermediateBase();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::control_matrix_t
CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::controlSecondDerivativeTerminal()
{
    return this->controlSecondDerivativeTerminalBase();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::control_state_matrix_t
CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::stateControlDerivativeIntermediate()
{
    return this->stateControlDerivativeIntermediateBase();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::control_state_matrix_t
CostFunctionAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::stateControlDerivativeTerminal()
{
    return this->stateControlDerivativeTerminalBase();
}

}  // namespace optcon
}  // namespace ct
