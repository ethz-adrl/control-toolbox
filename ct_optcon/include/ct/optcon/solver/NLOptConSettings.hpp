/***********************************************************************************
Copyright (c) 2017, Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo,
Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be used
      to endorse or promote products derived from this software without specific
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/

#ifndef INCLUDE_NLOPTCON_SETTINGS_HPP_
#define INCLUDE_NLOPTCON_SETTINGS_HPP_

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
struct LineSearchSettings {

	//! default constructor for the NLOptCon line-search settings
	LineSearchSettings ()
    {
        active = true;
        adaptive = false;
        maxIterations = 10;
        alpha_0 = 1.0;
        alpha_max = 1.0;
        n_alpha = 0.5;
    }

	//! check if the currently set line-search parameters are meaningful
    bool parametersOk() const {
        return (alpha_0 > 0.0) && (n_alpha > 0.0) && (n_alpha < 1.0) && (alpha_max > 0.0);
    }

    bool active;   /*!< Flag whether or not to perform line search */
    bool adaptive; /*!< Flag whether alpha_0 gets updated based on previous iteration */
    size_t maxIterations;  /*!< Maximum number of iterations during line search */
    double alpha_0;    /*!< Initial step size for line search. Use 1 for step size as suggested by NLOptCon */
    double alpha_max;    /*!< Maximum step size for line search. This is the limit when adapting alpha_0. */
    double n_alpha; /*!< Factor by which the line search step size alpha gets multiplied with after each iteration. Usually 0.5 is a good value. */


    //! print the current line search settings to console
    void print()
    {
        std::cout<<"Line Search Settings: "<<std::endl;
        std::cout<<"=====================" <<std::endl;
        std::cout<<"active: "<<active<<std::endl;
        std::cout<<"adaptive: "<<adaptive<<std::endl;
        std::cout<<"maxIterations: "<<maxIterations<<std::endl;
        std::cout<<"alpha_0: "<<alpha_0<<std::endl;
        std::cout<<"alpha_max: "<<alpha_max<<std::endl;
        std::cout<<"n_alpha: "<<n_alpha<<std::endl<<std::endl;
    }

    //! load line search settings from file
    void load(const std::string& filename, bool verbose = true, const std::string& ns = "line_search")
	{
		if (verbose)
			std::cout << "Trying to load line search settings from "<<filename<<": "<<std::endl;

		boost::property_tree::ptree pt;
		boost::property_tree::read_info(filename, pt);

		active = pt.get<bool>(ns +".active");
		maxIterations = pt.get<size_t>(ns +".maxIterations");
		alpha_0 = pt.get<double>(ns +".alpha_0");
		n_alpha = pt.get<double>(ns +".n_alpha");
		alpha_max = alpha_0;
		adaptive = false;

		try {
			adaptive = pt.get<bool>(ns +".adaptive");
			alpha_max = pt.get<double>(ns +".alpha_max");
		} catch (...) {}

		if (verbose)
		{
			std::cout << "Loaded line search settings from "<<filename<<": "<<std::endl;
			print();
		}
	}
};

struct ParallelBackwardSettings {

	ParallelBackwardSettings()
    {
        enabled = false;
        showWarnings = false;
        pollingTimeoutUs = 100;
    }

    bool parametersOk(size_t nThreadsTotal) const {
    	if (pollingTimeoutUs < 10)
    	{
    		std::cout << "Polling timeout is smaller than 10 us. Consider increasing it."<<std::endl;
    	}
        return (pollingTimeoutUs >= 10);
    }

    bool enabled;   /*!< Flag whether to use parallel backward */
    bool showWarnings; /*!< Show speed warnings if cost or linearization threads are not fast enough */
	size_t pollingTimeoutUs;

    void print()
    {
        std::cout<<"Parallel Backward Settings: "<<std::endl;
        std::cout<<"=====================" <<std::endl;
        std::cout<<"enabled: "<<enabled<<std::endl;
        std::cout<<"showWarnings: "<<showWarnings<<std::endl;
        std::cout<<"pollingTimeoutUs: "<<pollingTimeoutUs<<std::endl;
    }

    void load(const std::string& filename, bool verbose = true, const std::string& ns = "parallel_backward_pass")
	{
		if (verbose)
			std::cout << "Trying to load parallel backward pass settings from "<<filename<<": "<<std::endl;

		boost::property_tree::ptree pt;
		boost::property_tree::read_info(filename, pt);

		try {
			enabled = pt.get<bool>(ns +".enabled");
		} catch (...) {}
		try {
			showWarnings = pt.get<bool>(ns +".showWarnings");
		} catch (...) {}
		try {
			pollingTimeoutUs = pt.get<size_t>(ns +".pollingTimeoutUs");
		} catch (...) {}

		if (verbose)
		{
			std::cout << "Loaded parallel backward settings from "<<filename<<": "<<std::endl;
			print();
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
    //! enum indicating which integrator to use for the NLOptCon forward rollout
    enum INTEGRATOR { EULER = 0, RK4 = 1, EULER_SYM = 2, RK_SYM = 3};
    enum DISCRETIZATION { FORWARD_EULER = 0, BACKWARD_EULER = 1, TUSTIN = 2, MATRIX_EXPONENTIAL = 3 };

    enum SOLVER
    {
    	GNMS = 0,
    	ILQR	// todo: include more
    };

    //! NLOptCon Settings default constructor
    /*!
     * sets all settings to default values.
     */
    NLOptConSettings() :
    	integrator(RK4),
        discretization(BACKWARD_EULER),
        solver(GNMS),
		epsilon(1e-5),
		dt(0.001),
		dt_sim(0.001),
		min_cost_improvement(1e-4), // cost needs to be at least 1e-4 better before we assume convergence
		maxDefectSum(1e-5),
		max_iterations(100),
		fixedHessianCorrection(false),
		recordSmallestEigenvalue(false),
		nThreads(4),
		nThreadsEigen(1),
    	lineSearchSettings(),
		parallelBackward()
    {
    }

    INTEGRATOR integrator;	//! which integrator to use during the NLOptCon forward rollout
	DISCRETIZATION discretization;    
	SOLVER solver;    		//! which solver is to be used
	double epsilon;			//! Eigenvalue correction factor for Hessian regularization
    double dt;				//! sampling time for the control input (seconds)
    double dt_sim;			//! sampling time for the forward simulation (seconds) \warning dt_sim needs to be an integer multiple of dt.
    double min_cost_improvement;	//! minimum cost improvement between two interations to assume convergence
    double maxDefectSum;	//! maximum sum of squared defects (assume covergence if lower than this number)
    int max_iterations;		//! the maximum admissible number of NLOptCon main iterations \warning make sure to select this number high enough allow for convergence
    bool fixedHessianCorrection; //! perform Hessian regularization by incrementing the eigenvalues by epsilon.
    bool recordSmallestEigenvalue;	//! save the smallest eigenvalue of the Hessian
    size_t nThreads; //! number of threads, for MP version
    size_t nThreadsEigen; //! number of threads for eigen parallelization (applies both to MP and standard)
    LineSearchSettings lineSearchSettings; //! the line search settings
	ParallelBackwardSettings parallelBackward; //! do the backward pass in parallel with building the LQ problems (experimental)


    //! compute the number of discrete time steps for the current optimal control problem
    /*!
     *
     * @param timeHorizon the time horizon set in the optimal control problem
     * @return the new number of steps, minimum 1 steps long
     */
    int computeK(double timeHorizon) const
    {
    	if (timeHorizon < 0.0) { throw std::runtime_error("time Horizon is negative"); }
    	return std::max(1, (int)std::lround(timeHorizon / dt));
    }


    //! print the current NLOptCon settings to console
    void print()
    {
        std::cout<<"NLOptCon Settings: "<<std::endl;
        std::cout<<"==============="<<std::endl;
        std::cout<<"integrator: "<<integratorToString[integrator]<<std::endl;
        std::cout<<"discretization: " << discretizationToString[discretization]<<std::endl;
        std::cout<<"solver: " << solverToString[solver]<<std::endl;
        std::cout<<"dt: "<<dt<<std::endl;
        std::cout<<"dt_sim: "<<dt_sim<<std::endl;
        std::cout<<"maxIter: "<<max_iterations<<std::endl;
        std::cout<<"fixedHessianCorrection: "<<fixedHessianCorrection<<std::endl;
        std::cout<<"recordSmallestEigenvalue: "<<recordSmallestEigenvalue<<std::endl;
        std::cout<<"epsilon: "<<epsilon<<std::endl;
        std::cout<<"nThreads: "<<nThreads<<std::endl;
        std::cout<<"nThreadsEigen: "<<nThreadsEigen<<std::endl<<std::endl;

        lineSearchSettings.print();

        std::cout<<std::endl;

        parallelBackward.print();
    }

    //! perform a quick check if the given NLOptCon settings fulfil minimum requirements
    /*!
     * \warning This check cannot guarantee that the control problem is well parameterized
     */
    bool parametersOk() const
    {
    	if (dt == 0 || dt_sim == 0)
		{
			std::cout << "Either NLOptCon or simulation timestep is zero." << std::endl;
			return false;
		}

		if (std::fmod(dt, dt_sim) > 1e-6)
		{
			std::cout << "NLOptCon freqency dt should be a multiple of integration frequency dt_sim." << std::endl;
			return false;
		}

		if (nThreads > 100 || nThreadsEigen > 100)
		{
			std::cout << "Number of threads should not exceed 100." << std::endl;
			return false;
		}
		return (lineSearchSettings.parametersOk() && parallelBackward.parametersOk(nThreads));
    }


    //! load NLOptCon Settings from file
    /*!
     *
     * @param filename path to the settings file
     * @param verbose print out settings after reading them
     * @param ns (optional) namspace of parameter file
     */
    void load(const std::string& filename, bool verbose = true, const std::string& ns = "ilqg")
    {
    	if (verbose)
    		std::cout << "Trying to load NLOptCon config from "<<filename<<": "<<std::endl;

		boost::property_tree::ptree pt;
		boost::property_tree::read_info(filename, pt);

		try {
			epsilon = pt.get<double>(ns +".epsilon");
		} catch (...)
		{}

		dt = pt.get<double>(ns +".dt");
		dt_sim = pt.get<double>(ns +".dt_sim");
		min_cost_improvement = pt.get<double>(ns +".min_cost_improvement");
		maxDefectSum = pt.get<double>(ns +".maxDefectSum");
		max_iterations = pt.get<int>(ns +".max_iterations");

		try {
			nThreads = pt.get<size_t>(ns +".nThreads");

			std::string integratorStr = pt.get<std::string>(ns + ".integrator");
			if (stringToIntegrator.find(integratorStr) != stringToIntegrator.end())
			{
				integrator = stringToIntegrator[integratorStr];
			}
			else
			{
				std::cout << "Invalid integrator specified in config, should be one of the following:" << std::endl;

				for(auto it = stringToIntegrator.begin(); it != stringToIntegrator.end(); it++)
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

                for(auto it = stringToDiscretization.begin(); it != stringToDiscretization.end(); it++)
                {
                    std::cout << it->first << std::endl;
                }

                exit(-1);
            }

            std::string solverStr = pt.get<std::string>(ns + ".solver");
            if (solverToDiscretization.find(solverStr) != stringToSolver.end())
            {
            	solver = stringToSolver[solverStr];
            }
            else
            {
            	std::cout << "Invalid solver specified in config, should be one of the following:" << std::endl;

            	for(auto it = stringToSolver.begin(); it != stringToSolver.end(); it++)
            	{
            		std::cout << it->first << std::endl;
            	}

            	exit(-1);
            }

		} catch (...)
		{}

		try {
			nThreadsEigen = pt.get<size_t>(ns + ".nThreadsEigen");
		} catch (...)
		{}

		try {
			recordSmallestEigenvalue = pt.get<bool>(ns + ".recordSmallestEigenvalue");
		} catch (...)
		{}

		try {
			fixedHessianCorrection = pt.get<bool>(ns + ".fixedHessianCorrection");
		} catch (...)
		{}

		if (verbose)
		{
			std::cout << "Loaded NLOptCon config from "<<filename<<": "<<std::endl;
			print();
		}

		lineSearchSettings.load(filename, verbose, ns+".line_search");
		parallelBackward.load(filename, verbose, ns+".parallel_backward_pass");
    }


    //! load settings from config file and return as settings struct
    /*!
     * @param filename the path to the settings file
     * @param verbose print settings
     * @param ns (optional) settings namespace
     * @return the newly generated settings struct
     */
    static NLOptConSettings fromConfigFile(const std::string& filename, bool verbose = true, const std::string& ns = "ilqg")
    {
    	NLOptConSettings settings;
    	settings.load(filename, true, ns);
    	return settings;
    }

private:
    std::map<INTEGRATOR, std::string> integratorToString = {{EULER, "Euler"}, {RK4 , "Runge-Kutta 4th order"}, {EULER_SYM, "Symplectic Euler"}, {RK_SYM, "Symplectic Runge Kutta"}};
    std::map<std::string, INTEGRATOR> stringToIntegrator = {{"Euler", EULER}, {"RK4", RK4}, {"Euler_Sym", EULER_SYM}, {"Rk_Sym", RK_SYM}};

    std::map<DISCRETIZATION, std::string> discretizationToString = {{FORWARD_EULER, "Forward_euler"}, {BACKWARD_EULER, "Backward_euler"}, {TUSTIN, "Tustin"}};
    std::map<std::string, DISCRETIZATION> stringToDiscretization = {{"Forward_euler", FORWARD_EULER}, {"Backward_euler", BACKWARD_EULER}, {"Tustin", TUSTIN}};

    std::map<SOLVER, std::string> solverToString = {{GNMS, "GNMS"}, {ILQR, "ILQR"}};
    std::map<std::string, SOLVER> stringToSolver = {{"GNMS", GNMS}, {"ILQR", ILQR}};

};


}
}



#endif /* INCLUDE_GNMS_SETTINGS_HPP_ */
