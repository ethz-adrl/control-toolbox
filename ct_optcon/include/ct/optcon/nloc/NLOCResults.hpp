/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#ifdef MATLAB
#include <ct/optcon/matlab.hpp>
#endif

/*!
 * to do log every xth iter
 *
 * \todo also log settings here
 */
template <typename SCALAR = double>
struct SummaryAllIterations
{
    //! log of the iterations
    std::vector<size_t> iterations;

    //! different overall defect norms
    std::vector<SCALAR> defect_l1_norms;
    std::vector<SCALAR> defect_l2_norms;
    std::vector<SCALAR> e_box_norms;
    std::vector<SCALAR> e_gen_norms;

    //! step update norms
    std::vector<SCALAR> lx_norms;
    std::vector<SCALAR> lu_norms;

    //! costs and merits
    std::vector<SCALAR> intermediateCosts;
    std::vector<SCALAR> finalCosts;
    std::vector<SCALAR> totalCosts;
    std::vector<SCALAR> merits;

    //! step size
    std::vector<SCALAR> stepSizes;

    //! smallest eigenvalues
    std::vector<SCALAR> smallestEigenvalues;

    //! print summary of the last iteration with desired numeric precision
    template <int NUM_PRECISION = 12>
    void printSummaryLastIteration()
    {
        std::cout << "NLOC Summary of iteration " << iterations.back() << std::endl;
        std::cout << "==============================" << std::endl;

        std::cout << std::setprecision(NUM_PRECISION) << "interm. cost:\t" << intermediateCosts.back() << std::endl;
        std::cout << std::setprecision(NUM_PRECISION) << "final cost:\t" << finalCosts.back() << std::endl;
        std::cout << std::setprecision(NUM_PRECISION) << "total cost:\t" << totalCosts.back() << std::endl;
        std::cout << std::setprecision(NUM_PRECISION) << "total merit:\t" << merits.back() << std::endl;
        std::cout << std::setprecision(NUM_PRECISION) << "tot. defect L1:\t" << defect_l1_norms.back() << std::endl;
        std::cout << std::setprecision(NUM_PRECISION) << "tot. defect L2:\t" << defect_l2_norms.back() << std::endl;
        std::cout << std::setprecision(NUM_PRECISION) << "box cstr.err:\t" << e_box_norms.back() << std::endl;
        std::cout << std::setprecision(NUM_PRECISION) << "gen cstr.err:\t" << e_gen_norms.back() << std::endl;
        std::cout << std::setprecision(NUM_PRECISION) << "total lx norm:\t" << lx_norms.back() << std::endl;
        std::cout << std::setprecision(NUM_PRECISION) << "total lu norm:\t" << lu_norms.back() << std::endl;
        std::cout << std::setprecision(NUM_PRECISION) << "step-size:\t" << stepSizes.back() << std::endl;
        std::cout << "                   ===========" << std::endl;
        std::cout << std::endl;
    }


    void logToMatlab(const std::string& fileName)
    {
#ifdef MATLAB
        std::cout << "Logging NLOC summary to Matlab" << std::endl;
        matFile_.open(fileName + ".mat");

        matFile_.put("iterations", iterations);
        matFile_.put("defect_l1_norms", defect_l1_norms);
        matFile_.put("defect_l2_norms", defect_l2_norms);
        matFile_.put("box_constr_norms", e_box_norms);
        matFile_.put("gen_constr_norms", e_gen_norms);
        matFile_.put("lx_norms", lx_norms);
        matFile_.put("lu_norms", lu_norms);
        matFile_.put("intermediateCosts", intermediateCosts);
        matFile_.put("finalCosts", finalCosts);
        matFile_.put("totalCosts", totalCosts);
        matFile_.put("merits", merits);
        matFile_.put("stepSizes", stepSizes);
        matFile_.put("smallestEigenvalues", smallestEigenvalues);
        matFile_.close();
#endif
    }

//! if building with MATLAB support, include matfile
#ifdef MATLAB
    matlab::MatFile matFile_;
#endif  //MATLAB
};
