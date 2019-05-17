/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <map>

#include <boost/property_tree/info_parser.hpp>

#include <ct/optcon/nlp/solver/NlpSolverSettings.h>

namespace ct {
namespace optcon {


/**
 * @ingroup    DMS
 *
 * @brief      Defines the DMS settings
 */
class DmsSettings
{
public:
    typedef enum SplineType { ZERO_ORDER_HOLD = 0, PIECEWISE_LINEAR = 1, num_types_splining } SplineType_t;
    typedef enum ObjectiveType { KEEP_TIME_AND_GRID = 0, OPTIMIZE_GRID = 1, num_types_objectives } ObjectiveType_t;
    typedef enum IntegrationType { EULER = 0, RK4 = 1, RK5 = 2, num_types_integration } IntegrationType_t;
    typedef enum CostEvaluationType { SIMPLE = 0, FULL = 1, num_types_costevaluation } CostEvaluationType_t;

    /**
	 * @brief      Default constructor. Sets some default DMS settings. Note
	 *             that the optimal settings are strongly dependent on the
	 *             problem and it is highly recommended to use custom settings
	 */
    DmsSettings()
        : N_(30),
          T_(5),
          nThreads_(1),
          splineType_(ZERO_ORDER_HOLD),
          costEvaluationType_(SIMPLE),
          objectiveType_(KEEP_TIME_AND_GRID),
          h_min_(0.1),
          integrationType_(RK4),
          dt_sim_(0.01),
          absErrTol_(1e-10),
          relErrTol_(1e-10)
    {
    }

    size_t N_;                                 // the number of shots
    double T_;                                 // the time horizon
    size_t nThreads_;                          // number of threads
    SplineType_t splineType_;                  // spline interpolation type between the nodes
    CostEvaluationType_t costEvaluationType_;  // the the of costevaluator
    ObjectiveType_t objectiveType_;            // Timegrid optimization on(expensive) or off?
    double h_min_;                             // minimum admissible distance between two nodes in [sec]
    IntegrationType_t integrationType_;        // the integration type between the nodes
    double dt_sim_;                            // and the corresponding integration timestep
    double absErrTol_;                         // the absolute and relative integrator tolerances when using RK5
    double relErrTol_;

    NlpSolverSettings solverSettings_;
    ct::core::DerivativesCppadSettings cppadSettings_;

    void print()
    {
        std::cout << "=============================================================" << std::endl;
        std::cout << "\tMultiple Shooting Settings: " << std::endl;
        std::cout << "=============================================================" << std::endl;

        std::cout << "Shooting intervals N : " << N_ << std::endl;
        std::cout << "Total Time horizon: " << T_ << "s" << std::endl;
        std::cout << "Number of threads: " << nThreads_ << std::endl;
        std::cout << "Splinetype: " << splineToString[splineType_] << std::endl;
        std::cout << "Cost eval: " << costEvalToString[costEvaluationType_] << std::endl;
        std::cout << "Objective type: " << objTypeToString[objectiveType_] << std::endl;
        std::cout << "Integration type: " << integratorToString[integrationType_] << std::endl;
        std::cout << "Simulation timestep dt_sim: " << dt_sim_ << std::endl;

        std::cout << "=============================================================" << std::endl;

        solverSettings_.print();
    }

    bool parametersOk() const
    {
        if (N_ < 1 || N_ > 1000)
            return false;

        if (T_ <= 0.0)
            return false;

        if (nThreads_ < 1 || nThreads_ > 32)
            return false;

        if (splineType_ < 0 || !(splineType_ < SplineType_t::num_types_splining))
            return false;

        if (costEvaluationType_ < 0 || !(costEvaluationType_ < CostEvaluationType_t::num_types_costevaluation))
            return false;

        if (objectiveType_ < 0 || !(objectiveType_ < ObjectiveType_t::num_types_objectives))
            return false;

        if (h_min_ < 0.01 || h_min_ > T_)
            return false;

        if (integrationType_ < 0 || !(integrationType_ < IntegrationType_t::num_types_integration))
            return false;

        if (dt_sim_ <= 0.0 || dt_sim_ > 100.0)
            return false;

        if (absErrTol_ <= 1e-20 || absErrTol_ > 1.0)
            return false;

        if (relErrTol_ <= 1e-20 || relErrTol_ > 1.0)
            return false;

        return solverSettings_.parametersOk();
    }

    void load(const std::string& filename, bool verbose = true, const std::string& ns = "dms")
    {
        if (verbose)
            std::cout << "Trying to load DMS config from " << filename << ": " << std::endl;

        boost::property_tree::ptree pt;
        boost::property_tree::read_info(filename, pt);

        N_ = pt.get<unsigned int>(ns + ".N");
        T_ = pt.get<double>(ns + ".T");
        nThreads_ = pt.get<unsigned int>(ns + ".nThreads");
        splineType_ = static_cast<SplineType_t>(pt.get<unsigned int>(ns + ".InterpolationType"));
        costEvaluationType_ = static_cast<CostEvaluationType_t>(pt.get<unsigned int>(ns + ".CostEvaluationType"));
        objectiveType_ = static_cast<ObjectiveType_t>(pt.get<unsigned int>(ns + ".ObjectiveType"));
        h_min_ = pt.get<double>(ns + ".h_min");

        integrationType_ = static_cast<IntegrationType_t>(pt.get<unsigned int>(ns + ".IntegrationType"));
        dt_sim_ = pt.get<double>(ns + ".dt_sim");
        absErrTol_ = pt.get<double>(ns + ".AbsErrTol");
        relErrTol_ = pt.get<double>(ns + ".RelErrTol");

        solverSettings_.load(filename, verbose, ns + ".solver");  // todo bring in again
        cppadSettings_.load(filename, verbose, ns + ".cppad");

        if (verbose)
        {
            std::cout << "Loaded DMS config from " << filename << ": " << std::endl;
            print();
        }
    }

private:
    std::map<SplineType, std::string> splineToString = {
        {ZERO_ORDER_HOLD, "Zero order hold"}, {PIECEWISE_LINEAR, "Piecewise Linear"}};
    std::map<ObjectiveType, std::string> objTypeToString = {
        {KEEP_TIME_AND_GRID, "Timegrid fix"}, {OPTIMIZE_GRID, "Timegrid Optimization On"}};
    std::map<IntegrationType, std::string> integratorToString = {
        {EULER, "Euler"}, {RK4, "Runge-Kutta 4th order"}, {RK5, "RK5 adaptive step size"}};
    std::map<CostEvaluationType, std::string> costEvalToString = {{SIMPLE, "Simple"}, {FULL, "Full"}};
};
}
}
