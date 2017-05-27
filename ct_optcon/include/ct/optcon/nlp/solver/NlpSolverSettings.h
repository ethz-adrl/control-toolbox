#ifndef CT_OPTCON_NLPSETTINGS
#define CT_OPTCON_NLPSETTINGS

#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>

namespace ct {
namespace optcon {

class SnoptSettings
{
public: 

	SnoptSettings() :
	scale_option_param_(0),
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
	{}

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

	void print()
	{
		std::cout << "To be filled with something" << std::endl;
	}

    bool parametersOk() const
    {
    	return true;	
    }

    void load(const std::string& filename, bool verbose = true, const std::string& ns = "dms.nlp.snopt") {}

private:


};

class IpoptSettings
{
public:
	IpoptSettings() : 
	tol_(1e-5),
	constr_viol_tol_(1e-3),
	max_iter_(10),
	restoTol_(1e-7),
	acceptableTol_(1e-7),
	restoAcceptableTol_( 1e-7),
	linear_scaling_on_demand_("yes"),
	hessian_approximation_("limited-memory"),
	nlp_scaling_method_("gradient-based"),
	printLevel_(5),
	print_user_options_("no"),
	print_frequency_iter_(10000),
	printInfoString_("yes"),
	derivativeTest_("first-order"),
	derivativeTestTol_(1e-4),
	derivativeTestPerturbation_(1e-8),
	point_perturbation_radius_(0.1),
	checkDerivativesForNaninf_("no"),
	derivativeTestPrintAll_("no"),
	linearSystemScaling_("mc19"),
	linear_solver_("ma57"),
	jacobianApproximation_("finite-difference-values")
	{}

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
	double print_frequency_iter_;
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

	void print()
	{
		std::cout << "To be filled with something" << std::endl;
	}

    bool parametersOk() const
    {
    	return true;
    }

    void load(const std::string& filename, bool verbose = true, const std::string& ns = "dms.nlp.ipopt") {}

private:
};

class NlpSolverSettings
{
public:
	typedef enum SolverType {IPOPT = 0,	SNOPT = 1, num_types_solver} SolverType_t;

	NlpSolverSettings() :
	solverType_(IPOPT)
	{}

    SolverType_t solverType_;
    SnoptSettings snoptSettings_;
    IpoptSettings ipoptSettings_;

    void print()
    {
    	std::cout << "Using nlp solver: " << solverToString[solverType_] << std::endl;
    	if(solverType_ == IPOPT)
    		ipoptSettings_.print();
    	else if(solverType_ == SNOPT)
    		snoptSettings_.print();
    }

    bool parametersOk() const
    {
    	if(solverType_ == IPOPT)
    		return ipoptSettings_.parametersOk();
    	else if(solverType_ == SNOPT)
    		return snoptSettings_.parametersOk();
    	else 
    		return false;
    }

    void load(const std::string& filename, bool verbose = true, const std::string& ns = "dms.nlp")
    {
		boost::property_tree::ptree pt;
		boost::property_tree::read_info(filename, pt);  
		if(solverType_ == IPOPT)
			ipoptSettings_.load(filename, verbose, ns + ".ipopt");
		else if(solverType_ == SNOPT)
			snoptSettings_.load(filename, verbose, ns + ".snopt");

		if (verbose)
		{
			std::cout << "Loaded DMS config from "<<filename<<": "<<std::endl;
			print();
		}
    }

private:
	std::map<SolverType, std::string> solverToString = {{IPOPT, "IPOPT"}, {SNOPT, "SNOPT"}};

};


// void checkNlpSettings(const nlpSolverSettings& settings)
// {
//     std::cout << "checking the settings" << std::endl;
// }

// void loadNlpSettings(const std::string& filename, nlpSolverSettings& settings)
// {
// 	boost::property_tree::ptree pt;
// 	boost::property_tree::read_info(filename, pt);

// 	settings.solverType_ = (SolverType_t) pt.get<unsigned int>("nlp.SolverType");
// 	settings.checkDerivatives_ = pt.get<bool> ("nlp.CheckDerivatives");
// 	settings.displayIterationOutput_ = pt.get<bool> ("nlp.DisplayIterationOutput");
// }


// void printNlpSettings(nlpSolverSettings settings)
// {
// 	switch(settings.solverType_)
// 	{
// 		case IPOPT:
// 		{
// 			std::cout << "Using IPOPT for solving the NLP" << std::endl;
// 			break;
// 		}
// 		case SNOPT:
// 		{
// 			std::cout << "Using SNOPT for solving the NLP" << std::endl;
// 			break;
// 		}
// 		default:
// 		{
// 			throw(std::runtime_error("Unknown NLP solver type, Exiting"));
// 			break;
// 		}
// 	}

// 	if(settings.checkDerivatives_)
// 		std::cout << "Checking NLP derivatives" << std::endl;
// 	else
// 		std::cout << "NOT Checking NLP derivatives" << std::endl;


} // namespace optcon
} // namespace ct

#endif //CT_OPTCON_NLPSETTINGS
