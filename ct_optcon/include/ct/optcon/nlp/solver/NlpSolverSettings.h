/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>

namespace ct {
namespace optcon {


/**
 * @brief      The available types of NLP solvers
 */
enum class NlpSolverType : uint8_t
{
    IPOPT = 0,
    SNOPT = 1,
    num_types_solver
};

/**
 * @ingroup    NLP
 *
 * @brief      SnoptSolver settings. Details about the parameters can be found
 *             in the SNOPT documentation
 */
class SnoptSettings
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**
	 * @brief      Default constructor, sets the parameters to default values
	 */
    SnoptSettings()
        : scale_option_param_(0),
          derivative_option_param_(1),
          verify_level_param_(-1),
          major_iteration_limit_param_(500),
          minor_iteration_limit_param_(500),
          iteration_limit_param_(5000),
          major_optimality_tolerance_param_(1e-6),
          major_feasibility_tolerance_param_(1e-6),
          minor_feasibility_tolerance_param_(1e-6),
          print_file_param_(0),
          minor_print_level_param_(0),
          major_print_level_param_(0),
          new_basis_file_param_(0),
          old_basis_file_param_(0),
          backup_basis_file_param_(0),
          line_search_tolerance_param_(0.9),
          crash_option_(3),
          hessian_updates_(5)
    {
    }

    int scale_option_param_;
    int derivative_option_param_;
    int verify_level_param_;
    int major_iteration_limit_param_;
    int minor_iteration_limit_param_;
    int iteration_limit_param_;
    double major_optimality_tolerance_param_;
    double major_feasibility_tolerance_param_;
    double minor_feasibility_tolerance_param_;
    int print_file_param_;
    int minor_print_level_param_;
    int major_print_level_param_;
    int new_basis_file_param_;
    int old_basis_file_param_;
    int backup_basis_file_param_;
    double line_search_tolerance_param_;
    int crash_option_;
    int hessian_updates_;

    /**
	 * @brief      Prints out information about the settings
	 */
    void print()
    {
        std::cout << "SNOPT settings:" << std::endl;
        std::cout << "Max Major Iterations: " << major_iteration_limit_param_ << std::endl;
        std::cout << "Max Minor Iterations: " << minor_iteration_limit_param_ << std::endl;
    }

    /**
     * @brief      Checks whether to settings are filled with meaningful values
     *
     * @return     Returns true of the parameters are ok
     */
    bool parametersOk() const { return true; }
    /**
     * @brief      Loads the settings from a .info file
     *
     * @param[in]  filename  The filename
     * @param[in]  verbose   True if parameters to be printed out
     * @param[in]  ns        The namespace in the .info file
     */
    void load(const std::string& filename, bool verbose = true, const std::string& ns = "dms.solver.snopt")
    {
        boost::property_tree::ptree pt;
        boost::property_tree::read_info(filename, pt);

        minor_iteration_limit_param_ = pt.get<unsigned int>(ns + ".MaxMinorIterations");
        major_iteration_limit_param_ = pt.get<unsigned int>(ns + ".MaxMajorIterations");
        minor_print_level_param_ = pt.get<unsigned int>(ns + ".MinorPrintLevelVerbosity");
        major_print_level_param_ = pt.get<unsigned int>(ns + ".MajorPrintLevelVerbosity");
        major_optimality_tolerance_param_ = pt.get<double>(ns + ".OptimalityTolerance");
    }
};

/**
 * @ingroup    NLP
 *
 * @brief      IPOPT settings. Details about the parameters can be found
 *             in the IPOPT documentation
 */
class IpoptSettings
{
public:
    /**
	 * @brief      Default constructor, sets the parameters to default values
	 */
    IpoptSettings()
        : tol_(1e-8),
          constr_viol_tol_(1e-4),
          max_iter_(200),
          restoTol_(1e-7),
          acceptableTol_(1e-6),
          restoAcceptableTol_(1e-7),
          linear_scaling_on_demand_("yes"),
          hessian_approximation_("limited-memory"),
          nlp_scaling_method_("gradient-based"),
          printLevel_(5),
          print_user_options_("no"),
          print_frequency_iter_(1),
          printInfoString_("no"),
          derivativeTest_("none"),
          derivativeTestTol_(1e-4),
          derivativeTestPerturbation_(1e-8),
          point_perturbation_radius_(10),
          checkDerivativesForNaninf_("no"),
          derivativeTestPrintAll_("no"),
          linearSystemScaling_("none"),
          linear_solver_("mumps"),
          jacobianApproximation_("finite-difference-values")
    {
    }

    double tol_;
    double constr_viol_tol_;
    int max_iter_;
    double restoTol_;
    double acceptableTol_;
    double restoAcceptableTol_;
    std::string linear_scaling_on_demand_;
    std::string hessian_approximation_;
    std::string nlp_scaling_method_;
    int printLevel_;
    std::string print_user_options_;
    int print_frequency_iter_;
    std::string printInfoString_;
    std::string derivativeTest_;
    double derivativeTestTol_;
    double derivativeTestPerturbation_;
    double point_perturbation_radius_;
    std::string checkDerivativesForNaninf_;
    std::string derivativeTestPrintAll_;
    std::string linearSystemScaling_;
    std::string linear_solver_;
    std::string jacobianApproximation_;

    /**
	 * @brief      Prints out information about the settings
	 */
    void print()
    {
        std::cout << "IPOPT SETTINGS: " << std::endl;
        std::cout << "Using " << hessian_approximation_ << " hessian approximation" << std::endl;
        std::cout << "MaxIterations: " << max_iter_ << std::endl;
    }

    /**
     * @brief      Checks whether to settings are filled with meaningful values
     *
     * @return     Returns true of the parameters are ok
     */
    bool parametersOk() const { return true; }
    /**
     * @brief      Loads the settings from a .info file
     *
     * @param[in]  filename  The filename
     * @param[in]  verbose   True if parameters to be printed out
     * @param[in]  ns        The namespace in the .info fil
     */
    void load(const std::string& filename, bool verbose = true, const std::string& ns = "dms.nlp.ipopt")
    {
        boost::property_tree::ptree pt;
        boost::property_tree::read_info(filename, pt);  //
        max_iter_ = pt.get<unsigned int>(ns + ".MaxIterations");
        bool checkDerivatives = pt.get<bool>(ns + ".CheckDerivatives");
        if (checkDerivatives)
            derivativeTest_ = "first-order";
        if (!checkDerivatives)
            derivativeTest_ = "none";
        bool exactHessian = pt.get<bool>(ns + ".ExactHessian");
        if (exactHessian)
            hessian_approximation_ = "exact";
        if (!exactHessian)
            hessian_approximation_ = "limited-memory";

        printLevel_ = pt.get<unsigned int>(ns + ".Verbosity");
        tol_ = pt.get<double>(ns + ".OptimalityTolerance");

        setParameterIfExists(pt, jacobianApproximation_, "jacobianApproximation", ns);
        setParameterIfExists(pt, restoAcceptableTol_, "restoAcceptableTol", ns);
        setParameterIfExists(pt, restoAcceptableTol_, "restoAcceptableTol", ns);
        setParameterIfExists(pt, constr_viol_tol_, "constr_viol_tol", ns);
        setParameterIfExists(pt, restoTol_, "restoTol", ns);
        setParameterIfExists(pt, acceptableTol_, "acceptableTol", ns);
        setParameterIfExists(pt, restoAcceptableTol_, "restoAcceptableTol", ns);
        setParameterIfExists(pt, linear_scaling_on_demand_, "linear_scaling_on_demand", ns);
        setParameterIfExists(pt, nlp_scaling_method_, "nlp_scaling_method", ns);
        setParameterIfExists(pt, print_user_options_, "print_user_options", ns);
        setParameterIfExists(pt, print_frequency_iter_, "print_frequency_iter_", ns);
        setParameterIfExists(pt, printInfoString_, "printInfoString", ns);
        setParameterIfExists(pt, derivativeTestTol_, "derivativeTestTol", ns);
        setParameterIfExists(pt, derivativeTestPerturbation_, "derivativeTestPerturbation", ns);
        setParameterIfExists(pt, point_perturbation_radius_, "point_perturbation_radius", ns);
        setParameterIfExists(pt, checkDerivativesForNaninf_, "checkDerivativesForNaninf", ns);
        setParameterIfExists(pt, derivativeTestPrintAll_, "derivativeTestPrintAll", ns);
        setParameterIfExists(pt, linear_solver_, "linear_solver", ns);
        setParameterIfExists(pt, linearSystemScaling_, "linearSystemScaling", ns);
        setParameterIfExists(pt, jacobianApproximation_, "jacobianApproximation", ns);
    }

    template <typename TYPE>
    static void setParameterIfExists(const boost::property_tree::ptree& pt,
        TYPE& param,
        const std::string& param_name,
        const std::string& ns)
    {
        const std::string full_param_name = ns + "." + param_name;
        boost::optional<const boost::property_tree::ptree&> child = pt.get_child_optional(full_param_name);
        if (child)
        {
            param = child->get_value<TYPE>();
        }
    }
};

/**
 * @ingroup    NLP
 *
 * @brief      Contains the NLP solver settings
 */
class NlpSolverSettings
{
public:
    /**
	 * @brief      Default constructor, set default settings
	 */
    NlpSolverSettings()
        : solverType_(NlpSolverType::IPOPT), useGeneratedCostGradient_(false), useGeneratedConstraintJacobian_(false)
    {
    }

    NlpSolverType solverType_;
    bool useGeneratedCostGradient_;
    bool useGeneratedConstraintJacobian_;
    SnoptSettings snoptSettings_;
    IpoptSettings ipoptSettings_;

    /**
     * @brief      Prints out settings
     */
    void print()
    {
        std::cout << "=============================================================" << std::endl;
        std::cout << "\tNLP Solver Settings: " << std::endl;
        std::cout << "=============================================================" << std::endl;

        std::cout << "Using nlp solver: " << solverToString[solverType_] << std::endl;
        if (useGeneratedCostGradient_)
            std::cout << "Using generated Cost Gradient" << std::endl;
        else
            std::cout << "Using analytical cost Gradient" << std::endl;
        if (useGeneratedConstraintJacobian_)
            std::cout << "Using generated Constraints Jacobian" << std::endl;
        else
            std::cout << "Using anlyitical Constraints Jacobian" << std::endl;

        if (solverType_ == NlpSolverType::IPOPT)
            ipoptSettings_.print();
        else if (solverType_ == NlpSolverType::SNOPT)
            snoptSettings_.print();
    }

    /**
     * @brief      Checks whether to settings are filled with meaningful values
     *
     * @return     Returns true of the parameters are ok
     */
    bool parametersOk() const
    {
        if (solverType_ == NlpSolverType::IPOPT)
            return ipoptSettings_.parametersOk();
        else if (solverType_ == NlpSolverType::SNOPT)
            return snoptSettings_.parametersOk();
        else
            return false;
    }

    /**
     * @brief      Loads the settings from a .info file
     *
     * @param[in]  filename  The filename
     * @param[in]  verbose   True if parameters to be printed out
     * @param[in]  ns        The namespace in the .info fil
     */
    void load(const std::string& filename, bool verbose = true, const std::string& ns = "solver")
    {
        boost::property_tree::ptree pt;
        boost::property_tree::read_info(filename, pt);

        solverType_ = static_cast<NlpSolverType>(pt.get<unsigned int>(ns + ".SolverType"));
        useGeneratedCostGradient_ = pt.get<bool>(ns + ".UseGeneratedCostGradient");
        useGeneratedConstraintJacobian_ = pt.get<bool>(ns + ".UseGeneratedConstraintJacobian");

        if (solverType_ == NlpSolverType::IPOPT)
            ipoptSettings_.load(filename, verbose, ns + ".ipopt");
        else if (solverType_ == NlpSolverType::SNOPT)
            snoptSettings_.load(filename, verbose, ns + ".snopt");

        if (verbose)
        {
            std::cout << "Loaded NLP Solver settings from " << filename << ": " << std::endl;
            print();
        }
    }

private:
    std::map<NlpSolverType, std::string> solverToString = {
        {NlpSolverType::IPOPT, "IPOPT"}, {NlpSolverType::SNOPT, "SNOPT"}};
};


}  // namespace optcon
}  // namespace ct
