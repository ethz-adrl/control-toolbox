/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <map>

#include <boost/property_tree/info_parser.hpp>

namespace ct {
namespace optcon {


//! GNMS Line Search Settings
/*!
 * \ingroup GNMS
 *
 * The Line Search Settings are part of the general settings struct and hold parameters to customize the line-search for the NLOptCon controller update.
 */
struct LineSearchSettings
{
    //! types of backtracking line-search
    enum TYPE
    {
        NONE = 0,      //! take full-step updates
        SIMPLE,        //! simple backtracking using cost or merit function
        ARMIJO,        //! backtracking including riccati matrix measure
        GOLDSTEIN,  //! backtracking including riccati matrix measure and defects
        NUM_TYPES
    };

    //! mappings for line-search types
    std::map<TYPE, std::string> lineSearchTypeToString = {{NONE, "NONE (take full-step updates with alpha=1.0)"},
        {SIMPLE, "Simple Backtracking with cost/merit"}, {ARMIJO, "ARMIJO-style Backtracking for single-shooting"},
        {GOLDSTEIN, "GOLDSTEIN backtracking using Riccati matrices"}};

    std::map<std::string, TYPE> stringToLineSearchType = {
        {"NONE", NONE}, {"SIMPLE", SIMPLE}, {"ARMIJO", ARMIJO}, {"GOLDSTEIN", GOLDSTEIN}};


    //! default constructor for the NLOptCon line-search settings
    LineSearchSettings()
        : type(NONE),
          adaptive(false),
          maxIterations(10),
          alpha_0(1.0),
          alpha_max(1.0),
          n_alpha(0.5),
          armijo_parameter(0.01),
          debugPrint(false)
    {
    }

    //! check if the currently set line-search parameters are meaningful
    bool parametersOk() const { return (alpha_0 > 0.0) && (n_alpha > 0.0) && (n_alpha < 1.0) && (alpha_max > 0.0); }
    TYPE type;            /*!< type of line search */
    bool adaptive;        /*!< Flag whether alpha_0 gets updated based on previous iteration */
    size_t maxIterations; /*!< Maximum number of iterations during line search */
    double alpha_0;       /*!< Initial step size for line search. Use 1 for step size as suggested by NLOptCon */
    double alpha_max;     /*!< Maximum step size for line search. This is the limit when adapting alpha_0. */
    double
        n_alpha; /*!< Factor by which the step size alpha gets scaled after each iteration. Usually 0.5 is a good value. */
    double armijo_parameter; /*!< "Control Parameter" in Armijo line search condition. */
    bool debugPrint;         /*!< Print out debug information during line-search*/


    //! print the current line search settings to console
    void print() const
    {
        std::cout << "Line Search Settings: " << std::endl;
        std::cout << "=====================" << std::endl;
        std::cout << "type:\t" << lineSearchTypeToString.at(type) << std::endl;
        std::cout << "adaptive:\t" << adaptive << std::endl;
        std::cout << "maxIter:\t" << maxIterations << std::endl;
        std::cout << "alpha_0:\t" << alpha_0 << std::endl;
        std::cout << "alpha_max:\t" << alpha_max << std::endl;
        std::cout << "n_alpha:\t" << n_alpha << std::endl;
        std::cout << "armijo_parameter:\t" << armijo_parameter << std::endl;
        std::cout << "debugPrint:\t" << debugPrint << std::endl;
        std::cout << "              =======" << std::endl;
        std::cout << std::endl;
    }

    //! load line search settings from file
    void load(const std::string& filename, bool verbose = true, const std::string& ns = "line_search")
    {
        if (verbose)
            std::cout << "Trying to load line search settings from " << filename << ": " << std::endl;

        boost::property_tree::ptree pt;
        boost::property_tree::read_info(filename, pt);

        std::string ls_type = pt.get<std::string>(ns + ".type");
        if (stringToLineSearchType.find(ls_type) != stringToLineSearchType.end())
        {
            type = stringToLineSearchType[ls_type];
        }
        else
        {
            std::cout << "Invalid line search type specified in config, should be one of the following:" << std::endl;

            for (auto it = stringToLineSearchType.begin(); it != stringToLineSearchType.end(); it++)
            {
                std::cout << it->first << std::endl;
            }

            exit(-1);
        }


        maxIterations = pt.get<size_t>(ns + ".maxIterations");
        alpha_0 = pt.get<double>(ns + ".alpha_0");
        n_alpha = pt.get<double>(ns + ".n_alpha");
        debugPrint = pt.get<bool>(ns + ".debugPrint");
        alpha_max = alpha_0;
        adaptive = false;

        try
        {
            armijo_parameter = pt.get<double>(ns + ".armijo_parameter");
        } catch (...)
        {
        }

        try
        {
            adaptive = pt.get<bool>(ns + ".adaptive");
        } catch (...)
        {
        }

        try
        {
            alpha_max = pt.get<double>(ns + ".alpha_max");
        } catch (...)
        {
        }

        if (verbose)
        {
            std::cout << "Loaded line search settings from " << filename << ": " << std::endl;
            print();
        }
    }
};


//! LQOC Solver settings
/*!
 * Settings for solving each linear-quadratic (constrained) sub-problem
 */
struct LQOCSolverSettings
{
public:
    LQOCSolverSettings() : lqoc_debug_print(false), num_lqoc_iterations(10) {}

    bool lqoc_debug_print;
    int num_lqoc_iterations;  //! number of allowed sub-iterations of LQOC solver per NLOC main iteration

    void print() const
    {
        std::cout << "======================= LQOCSolverSettings =====================" << std::endl;
        std::cout << "num_lqoc_iterations: \t" << num_lqoc_iterations << std::endl;
        std::cout << "lqoc_debug_print: \t" << lqoc_debug_print << std::endl;
    }

    void load(const std::string& filename, bool verbose = true, const std::string& ns = "lqoc_solver_settings")
    {
        if (verbose)
            std::cout << "Trying to load LQOCSolverSettings config from " << filename << ": " << std::endl;

        boost::property_tree::ptree pt;
        boost::property_tree::read_info(filename, pt);

        try
        {
            num_lqoc_iterations = pt.get<int>(ns + ".num_lqoc_iterations");
        } catch (...)
        {
        }
        try
        {
            lqoc_debug_print = pt.get<bool>(ns + ".lqoc_debug_print");
        } catch (...)
        {
        }
    }
};


/*!
 * \ingroup NLOptCon
 *
 * \brief Settings for the NLOptCon algorithm.
 */
class NLOptConSettings
{
public:
    //! algorithm types for solving the NLOC problem
    enum NLOCP_ALGORITHM
    {
        GNMS = 0,   //! Gauss-Newton Multiple Shooting (shooting interval = control interval)
        ILQR,       //! Classical iLQR (1 shooting interval equal to problem horizon)
        MS_ILQR,    //! multiple-shooting iLQR
        SS_OL,      //! Classical (open-loop) Single Shooting
        SS_CL,      //! Closed-loop single shooting
        GNMS_M_OL,  //! GNMS(M) with open-loop shooting
        GNMS_M_CL,  //! GNMS(M) with closed-loop shooting
        NUM_TYPES
    };

    //! the linear optimal control problem solver in the background
    enum LQOCP_SOLVER
    {
        GNRICCATI_SOLVER = 0,
        HPIPM_SOLVER = 1
    };

    using APPROXIMATION = typename core::SensitivityApproximationSettings::APPROXIMATION;

    //! NLOptCon Settings default constructor
    /*!
     * sets all settings to default values.
     */
    NLOptConSettings()
        : integrator(ct::core::IntegrationType::RK4),
          discretization(APPROXIMATION::BACKWARD_EULER),
          timeVaryingDiscretization(false),
          nlocp_algorithm(GNMS),
          lqocp_solver(GNRICCATI_SOLVER),
          loggingPrefix("alg"),
          epsilon(1e-5),
          dt(0.001),
          K_sim(1),                    //! by default, there is only one sub-integration step
          K_shot(1),                   //! by default the shot length is equal to the control length
          min_cost_improvement(1e-5),  //! cost needs to be at least 1e-5 better before we assume convergence
          maxDefectSum(1e-5),
          meritFunctionRho(0.0),
          meritFunctionRhoConstraints(1.0),
          max_iterations(100),
          fixedHessianCorrection(false),
          recordSmallestEigenvalue(false),
          nThreads(4),
          nThreadsEigen(4),
          lineSearchSettings(),
          debugPrint(false),
          printSummary(true),
          useSensitivityIntegrator(false),
          logToMatlab(false)
    {
    }

    ct::core::IntegrationType integrator;  //! which integrator to use during the NLOptCon forward rollout
    APPROXIMATION discretization;
    bool timeVaryingDiscretization;
    NLOCP_ALGORITHM nlocp_algorithm;  //! which nonlinear optimal control algorithm is to be used
    LQOCP_SOLVER lqocp_solver;        //! the solver for the linear-quadratic optimal control problem
    std::string loggingPrefix;        //! the prefix to be stored before the matfile name for logging
    double epsilon;                   //! Eigenvalue correction factor for Hessian regularization
    double dt;                        //! sampling time for the control input (seconds)
    int K_sim;                        //! number of sub-integration-steps
    int K_shot;                       //! duration of a shot as an integer multiple of dt
    double min_cost_improvement;      //! minimum cost improvement between two interations to assume convergence
    double maxDefectSum;              //! maximum sum of squared defects (assume covergence if lower than this number)
    double meritFunctionRho;          //! trade off between internal (defect)constraint violation and cost
    double meritFunctionRhoConstraints;  //! trade off between external (general and path) constraint violation and cost
    int max_iterations;  //! the maximum admissible number of NLOptCon main iterations \warning make sure to select this number high enough allow for convergence
    bool fixedHessianCorrection;    //! perform Hessian regularization by incrementing the eigenvalues by epsilon.
    bool recordSmallestEigenvalue;  //! save the smallest eigenvalue of the Hessian
    int nThreads;                   //! number of threads, for MP version
    size_t
        nThreadsEigen;  //! number of threads for eigen parallelization (applies both to MP and ST) Note. in order to activate Eigen parallelization, compile with '-fopenmp'
    LineSearchSettings lineSearchSettings;  //! the line search settings
    LQOCSolverSettings lqoc_solver_settings;
    bool debugPrint;
    bool printSummary;
    bool useSensitivityIntegrator;
    bool logToMatlab;  //! log to matlab (true/false)


    //! compute the number of discrete time steps for an arbitrary input time interval
    /*!
     * @param timeHorizon the time horizon of interest, e.g. overall optimal control time horizon or shot-length
     * @return the resulting number of steps, minimum 1 steps long
     *
     * \todo naming it K is confusing, should better be N.
     */
    int computeK(double timeHorizon) const
    {
        if (timeHorizon < 0.0)
        {
            throw std::runtime_error("time Horizon is negative");
        }
        return std::max(1, (int)std::lround(timeHorizon / dt));
    }

    //! compute the simulation timestep
    double getSimulationTimestep() const { return (dt / (double)K_sim); }

    //! return if this is a closed-loop shooting algorithm (or not)
    bool closedLoopShooting() const { return nlocAlgorithmToClosedLoopShooting.at(nlocp_algorithm); }

    //! return if this is a single-shooting algorithm (or not)
    bool isSingleShooting() const { return nlocAlgorithmToSingleShooting.at(nlocp_algorithm); }

    //! print the current NLOptCon settings to console
    void print() const
    {
        std::cout << "======================= NLOptCon Settings =====================" << std::endl;
        std::cout << "===============================================================" << std::endl;
        std::cout << "integrator: " << integratorToString.at(integrator) << std::endl;
        std::cout << "discretization: " << discretizationToString.at(discretization) << std::endl;
        std::cout << "time varying discretization: " << timeVaryingDiscretization << std::endl;
        std::cout << "nonlinear OCP algorithm: " << nlocAlgorithmToString.at(nlocp_algorithm) << std::endl;
        std::cout << "linear-quadratic OCP solver: " << lqocSolverToString.at(lqocp_solver) << std::endl;
        std::cout << "dt:\t" << dt << std::endl;
        std::cout << "K_sim:\t" << K_sim << std::endl;
        std::cout << "K_shot:\t" << K_shot << std::endl;
        std::cout << "maxIter:\t" << max_iterations << std::endl;
        std::cout << "min cost improvement:\t" << min_cost_improvement << std::endl;
        std::cout << "max defect sum:\t" << maxDefectSum << std::endl;
        std::cout << "merit function rho defects:\t" << meritFunctionRho << std::endl;
        std::cout << "merit function rho constraints:\t" << meritFunctionRhoConstraints << std::endl;
        std::cout << "fixedHessianCorrection:\t" << fixedHessianCorrection << std::endl;
        std::cout << "recordSmallestEigenvalue:\t" << recordSmallestEigenvalue << std::endl;
        std::cout << "epsilon:\t" << epsilon << std::endl;
        std::cout << "nThreads:\t" << nThreads << std::endl;
        std::cout << "nThreadsEigen:\t" << nThreadsEigen << std::endl;
        std::cout << "loggingPrefix:\t" << loggingPrefix << std::endl;
        std::cout << "debugPrint:\t" << debugPrint << std::endl;
        std::cout << "printSummary:\t" << printSummary << std::endl;
        std::cout << "useSensitivityIntegrator:\t" << useSensitivityIntegrator << std::endl;
        std::cout << "logToMatlab:\t" << logToMatlab << std::endl;
        std::cout << std::endl;

        lineSearchSettings.print();

        lqoc_solver_settings.print();

        std::cout << "===============================================================" << std::endl;
    }

    //! perform a quick check if the given NLOptCon settings fulfil minimum requirements
    /*!
     * \warning This check cannot guarantee that the control problem is well parameterized
     */
    bool parametersOk() const
    {
        if (dt <= 0)
        {
            std::cout << "Invalid parameter dt in NLOptConSettings, needs to be > 0. dt currently is " << dt
                      << std::endl;
            return false;
        }

        if (K_sim <= 0)
        {
            std::cout << "Invalid parameter K_sim in NLOptConSettings, needs to be >= 1. K_sim currently is " << K_sim
                      << std::endl;
            return false;
        }

        if (K_shot <= 0)
        {
            std::cout << "Invalid parameter K_shot in NLOptConSettings, needs to be >= 1. K_shot currently is "
                      << K_shot << std::endl;
            return false;
        }

        // TODO need thorough check if this is really the case ....
        if ((K_shot > 1) && (nlocp_algorithm == ILQR))
        {
            std::cout << "Invalid parameter: for iLQR K_shot needs to be 1. K_shot currently is " << K_shot
                      << std::endl;
            return false;
        }

        if (nThreads > 100 || nThreadsEigen > 100)
        {
            std::cout << "Number of threads should not exceed 100." << std::endl;
            return false;
        }
        return (lineSearchSettings.parametersOk());
    }


    //! load NLOptCon Settings from file
    /*!
     *
     * @param filename path to the settings file
     * @param verbose print out settings after reading them
     * @param ns (optional) namspace of parameter file
     */
    void load(const std::string& filename, bool verbose = true, const std::string& ns = "alg")
    {
        if (verbose)
            std::cout << "Trying to load NLOptCon config from " << filename << ": " << std::endl;

        boost::property_tree::ptree pt;
        boost::property_tree::read_info(filename, pt);

        try
        {
            epsilon = pt.get<double>(ns + ".epsilon");
        } catch (...)
        {
        }
        try
        {
            timeVaryingDiscretization = pt.get<bool>(ns + ".timeVaryingDiscretization");
        } catch (...)
        {
        }
        try
        {
            min_cost_improvement = pt.get<double>(ns + ".min_cost_improvement");
        } catch (...)
        {
        }
        try
        {
            maxDefectSum = pt.get<double>(ns + ".maxDefectSum");
        } catch (...)
        {
        }
        try
        {
            meritFunctionRho = pt.get<double>(ns + ".meritFunctionRho");
        } catch (...)
        {
        }
        try
        {
            meritFunctionRhoConstraints = pt.get<double>(ns + ".meritFunctionRhoConstraints");
        } catch (...)
        {
        }
        try
        {
            max_iterations = pt.get<int>(ns + ".max_iterations");
        } catch (...)
        {
        }
        try
        {
            nThreads = pt.get<int>(ns + ".nThreads");
        } catch (...)
        {
        }
        try
        {
            loggingPrefix = pt.get<std::string>(ns + ".loggingPrefix");
        } catch (...)
        {
        }
        try
        {
            debugPrint = pt.get<bool>(ns + ".debugPrint");
        } catch (...)
        {
        }
        try
        {
            printSummary = pt.get<bool>(ns + ".printSummary");
        } catch (...)
        {
        }
        try
        {
            useSensitivityIntegrator = pt.get<bool>(ns + ".useSensitivityIntegrator");
        } catch (...)
        {
        }
        try
        {
            logToMatlab = pt.get<bool>(ns + ".logToMatlab");
        } catch (...)
        {
        }
        try
        {
            dt = pt.get<double>(ns + ".dt");
        } catch (...)
        {
        }
        try
        {
            K_sim = pt.get<int>(ns + ".K_sim");
        } catch (...)
        {
        }
        try
        {
            K_shot = pt.get<int>(ns + ".K_shot");
        } catch (...)
        {
        }
        try
        {
            nThreadsEigen = pt.get<size_t>(ns + ".nThreadsEigen");
        } catch (...)
        {
        }
        try
        {
            recordSmallestEigenvalue = pt.get<bool>(ns + ".recordSmallestEigenvalue");
        } catch (...)
        {
        }
        try
        {
            fixedHessianCorrection = pt.get<bool>(ns + ".fixedHessianCorrection");
        } catch (...)
        {
        }

        try
        {
            lineSearchSettings.load(filename, verbose, ns + ".line_search");
        } catch (...)
        {
        }
        try
        {
            lqoc_solver_settings.load(filename, verbose, ns + ".lqoc_solver_settings");
        } catch (...)
        {
        }


        try
        {
            std::string integratorStr = pt.get<std::string>(ns + ".integrator");
            if (stringToIntegrator.find(integratorStr) != stringToIntegrator.end())
            {
                integrator = stringToIntegrator[integratorStr];
            }
            else
            {
                std::cout << "Invalid integrator specified in config, should be one of the following:" << std::endl;

                for (auto it = stringToIntegrator.begin(); it != stringToIntegrator.end(); it++)
                {
                    std::cout << it->first << std::endl;
                }

                exit(-1);
            }

            std::string discretizationStr = pt.get<std::string>(ns + ".discretization");
            if (stringToDiscretization.find(discretizationStr) != stringToDiscretization.end())
            {
                discretization = stringToDiscretization[discretizationStr];
            }
            else
            {
                std::cout << "Invalid discretization specified in config, should be one of the following:" << std::endl;

                for (auto it = stringToDiscretization.begin(); it != stringToDiscretization.end(); it++)
                {
                    std::cout << it->first << std::endl;
                }

                exit(-1);
            }

            std::string nlocp_algorithmStr = pt.get<std::string>(ns + ".nlocp_algorithm");
            if (stringToNlocAlgorithm.find(nlocp_algorithmStr) != stringToNlocAlgorithm.end())
            {
                nlocp_algorithm = stringToNlocAlgorithm[nlocp_algorithmStr];
            }
            else
            {
                std::cout << "Invalid nlocp_algorithm specified in config, should be one of the following:"
                          << std::endl;

                for (auto it = stringToNlocAlgorithm.begin(); it != stringToNlocAlgorithm.end(); it++)
                {
                    std::cout << it->first << std::endl;
                }

                exit(-1);
            }


            std::string locp_solverStr = pt.get<std::string>(ns + ".locp_solver");
            if (stringToLqocSolver.find(locp_solverStr) != stringToLqocSolver.end())
            {
                lqocp_solver = stringToLqocSolver[locp_solverStr];
            }
            else
            {
                std::cout << "Invalid locp_solver specified in config, should be one of the following:" << std::endl;

                for (auto it = stringToLqocSolver.begin(); it != stringToLqocSolver.end(); it++)
                {
                    std::cout << it->first << std::endl;
                }

                exit(-1);
            }

        } catch (...)
        {
        }

        if (verbose)
        {
            std::cout << "Loaded NLOptCon config from " << filename << ": " << std::endl;
            print();
        }
    }


    //! load settings from config file and return as settings struct
    /*!
     * @param filename the path to the settings file
     * @param verbose print settings
     * @param ns (optional) settings namespace
     * @return the newly generated settings struct
     */
    static NLOptConSettings fromConfigFile(const std::string& filename,
        bool verbose = true,
        const std::string& ns = "alg")
    {
        NLOptConSettings settings;
        settings.load(filename, true, ns);
        return settings;
    }

private:
    //! mappings for integrator types
    std::map<ct::core::IntegrationType, std::string> integratorToString = {{ct::core::IntegrationType::EULER, "Euler"},
        {ct::core::IntegrationType::RK4, "Runge-Kutta 4th Order"},
        {ct::core::IntegrationType::MODIFIED_MIDPOINT, "Modified midpoint"},
        {ct::core::IntegrationType::ODE45, "ode45"}, {ct::core::IntegrationType::RK5VARIABLE, "RK5 variable step"},
        {ct::core::IntegrationType::RK78, "RK78"}, {ct::core::IntegrationType::BULIRSCHSTOER, "Bulirsch-Stoer"},
        {ct::core::IntegrationType::EULERCT, "Euler (CT)"},
        {ct::core::IntegrationType::RK4CT, "Runge-Kutta 4th Order (CT"},
        {ct::core::IntegrationType::EULER_SYM, "Symplectic Euler"},
        {ct::core::IntegrationType::RK_SYM, "Symplectic Runge Kutta"}};

    std::map<std::string, ct::core::IntegrationType> stringToIntegrator = {{"Euler", ct::core::IntegrationType::EULER},
        {"RK4", ct::core::IntegrationType::RK4}, {"MODIFIED_MIDPOINT", ct::core::IntegrationType::MODIFIED_MIDPOINT},
        {"ODE45", ct::core::IntegrationType::ODE45}, {"RK5VARIABLE", ct::core::IntegrationType::RK5VARIABLE},
        {"RK78", ct::core::IntegrationType::RK78}, {"BULIRSCHSTOER", ct::core::IntegrationType::BULIRSCHSTOER},
        {"EulerCT", ct::core::IntegrationType::EULERCT}, {"RK4CT", ct::core::IntegrationType::RK4CT},
        {"Euler_Sym", ct::core::IntegrationType::EULER_SYM}, {"Rk_Sym", ct::core::IntegrationType::RK_SYM}};


    //! mappings for discretization types
    std::map<APPROXIMATION, std::string> discretizationToString = {{APPROXIMATION::FORWARD_EULER, "Forward_euler"},
        {APPROXIMATION::BACKWARD_EULER, "Backward_euler"}, {APPROXIMATION::SYMPLECTIC_EULER, "Symplectic_euler"},
        {APPROXIMATION::TUSTIN, "Tustin"}, {APPROXIMATION::MATRIX_EXPONENTIAL, "Matrix_exponential"}};

    std::map<std::string, APPROXIMATION> stringToDiscretization = {{"Forward_euler", APPROXIMATION::FORWARD_EULER},
        {"Backward_euler", APPROXIMATION::BACKWARD_EULER}, {"Symplectic_euler", APPROXIMATION::SYMPLECTIC_EULER},
        {"Tustin", APPROXIMATION::TUSTIN}, {"Matrix_exponential", APPROXIMATION::MATRIX_EXPONENTIAL}};


    //! mappings for algorithm types
    std::map<NLOCP_ALGORITHM, std::string> nlocAlgorithmToString = {{GNMS, "GNMS (Gauss-Newton Multiple Shooting)"},
        {ILQR, "ILQR (iterative linear-quadratic optimal control)"}, {MS_ILQR, "MS_ILQR (multiple-shooting iLQR)"},
        {SS_OL, "SS_OL (open-loop Single Shooting)"}, {SS_CL, "SS_CL (closed-loop Single Shooting)"},
        {GNMS_M_OL, "GNMS_M_OL (GNMS(M) with open-loop shooting)"},
        {GNMS_M_CL, "GNMS_M_CL (GNMS(M) with closed-loop shooting)"}};

    std::map<std::string, NLOCP_ALGORITHM> stringToNlocAlgorithm = {{"GNMS", GNMS}, {"ILQR", ILQR},
        {"MS_ILQR", MS_ILQR}, {"SS_OL", SS_OL}, {"SS_CL", SS_CL}, {"GNMS_M_OL", GNMS_M_OL}, {"GNMS_M_CL", GNMS_M_CL}};

    std::map<NLOCP_ALGORITHM, bool> nlocAlgorithmToClosedLoopShooting = {{GNMS, false}, {ILQR, true}, {MS_ILQR, true},
        {SS_OL, false}, {SS_CL, true}, {GNMS_M_OL, false}, {GNMS_M_CL, true}};

    std::map<NLOCP_ALGORITHM, bool> nlocAlgorithmToSingleShooting = {{GNMS, false}, {ILQR, true}, {MS_ILQR, false},
        {SS_OL, true}, {SS_CL, true}, {GNMS_M_OL, false}, {GNMS_M_CL, false}};

    //! mappings for linear-quadratic solver types
    std::map<LQOCP_SOLVER, std::string> lqocSolverToString = {
        {GNRICCATI_SOLVER, "GNRICCATI_SOLVER"}, {HPIPM_SOLVER, "HPIPM_SOLVER"}};

    std::map<std::string, LQOCP_SOLVER> stringToLqocSolver = {
        {"GNRICCATI_SOLVER", GNRICCATI_SOLVER}, {"HPIPM_SOLVER", HPIPM_SOLVER}};
};
}  // namespace optcon
}  // namespace ct
