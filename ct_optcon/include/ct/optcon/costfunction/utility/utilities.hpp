/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <iostream>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>

#include <ct/optcon/costfunction/term/TermBase.hpp>

#include "../CostFunction.hpp"

namespace ct {
namespace optcon {

template <typename SCALAR>
void loadScalarCF(const std::string& filename,
    const std::string& scalarName,
    SCALAR& scalar,
    const std::string& termName = "")
{
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(filename, pt);

    scalar = pt.get<SCALAR>(termName + ".weights." + scalarName);
}

template <typename SCALAR>
void loadScalarOptionalCF(const std::string& filename,
    const std::string& scalarName,
    SCALAR& scalar,
    const std::string& termName,
    const SCALAR& defaultValue)
{
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(filename, pt);

    scalar = pt.get<SCALAR>(termName + ".weights." + scalarName, defaultValue);
}

template <typename SCALAR, int ROW, int COL>
void loadMatrixCF(const std::string& filename,
    const std::string& matrixName,
    Eigen::Matrix<SCALAR, ROW, COL>& matrix,
    const std::string& termName = "")
{
    size_t rows = matrix.rows();
    size_t cols = matrix.cols();

    boost::property_tree::ptree pt;
    boost::property_tree::read_info(filename, pt);

    double scaling = pt.get<double>(termName + ".weights." + matrixName + ".scaling", 1);

    for (size_t i = 0; i < rows; i++)
    {
        for (size_t j = 0; j < cols; j++)
        {
            if (termName == "")
            {
                matrix(i, j) =
                    scaling *
                    pt.get<double>(matrixName + "." + "(" + std::to_string(i) + "," + std::to_string(j) + ")", 0.0);
            }
            else
            {
                matrix(i, j) = scaling *
                               pt.get<double>(termName + ".weights." + matrixName + "." + "(" + std::to_string(i) +
                                                  "," + std::to_string(j) + ")",
                                   0.0);
            }
        }
    }
}

template <typename TERM_PTR, typename costFuncType>
void addTerm(const std::string& filename,
    std::string& currentTerm,
    int currentTermType,
    TERM_PTR term,
    costFuncType* costFunc,
    bool verbose = false)
{
    switch (currentTermType)
    {
        case 0:
            costFunc->addIntermediateTerm(term, verbose);
            break;
        case 1:
            costFunc->addFinalTerm(term, verbose);
            break;
        default:
            if (verbose)
            {
                std::cout << "error code 1 => term type other than term0 and term1 encountered" << std::endl;
            }
            BOOST_PROPERTY_TREE_THROW(boost::property_tree::info_parser::info_parser_error(
                "read error code = ", "", 1));  //error code 1 => term type otherthan term0 and term1 encountered
            break;
    }
    term->loadTimeActivation(filename, currentTerm, verbose);
    term->loadConfigFile(filename, currentTerm, verbose);

    if (verbose)
        std::cout << "Successfully loaded term " + currentTerm << std::endl;
}

template <typename TERM_PTR, typename costFuncType>
void addADTerm(const std::string& filename,
    std::string& currentTerm,
    int currentTermType,
    TERM_PTR term,
    costFuncType* costFunc,
    bool verbose = false)
{
    switch (currentTermType)
    {
        case 0:
            costFunc->addIntermediateADTerm(term, verbose);
            break;
        case 1:
            costFunc->addFinalADTerm(term, verbose);
            break;
        default:
            if (verbose)
            {
                std::cout << "error code 1 => term type other than term0 and term1 encountered" << std::endl;
            }
            BOOST_PROPERTY_TREE_THROW(boost::property_tree::info_parser::info_parser_error(
                "read error code = ", "", 1));  //error code 1 => term type otherthan term0 and term1 encountered
            break;
    }
    term->loadTimeActivation(filename, currentTerm, verbose);
    term->loadConfigFile(filename, currentTerm, verbose);

    if (verbose)
        std::cout << "Successfully loaded term " + currentTerm << std::endl;
}

}  // namespace optcon
}  // namespace ct
