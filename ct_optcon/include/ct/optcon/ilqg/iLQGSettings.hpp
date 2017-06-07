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

#ifndef INCLUDE_ILQG_SETTINGS_HPP_
#define INCLUDE_ILQG_SETTINGS_HPP_

#include <map>

#include <boost/property_tree/info_parser.hpp>

namespace ct {
namespace optcon {


//! iLQG Line Search Settings
/*!
 * \ingroup iLQG
 *
 * The Line Search Settings are part of the general settings struct and hold parameters to customize the line-search for the iLQG controller update.
 */
struct iLQGLineSearchSettings {

	//! default constructor for the iLQG line-search settings
	iLQGLineSearchSettings ()
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
    double alpha_0;    /*!< Initial step size for line search. Use 1 for step size as suggested by iLQG */
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



/*!
 * \ingroup iLQG
 *
 * \brief Settings for the iLQG algorithm.
 */
class iLQGSettings
{
public:

	//! enum indicating which integrator to use for the iLQG forward rollout
    enum INTEGRATOR {
    	EULER = 0,  //! trivial fixed time-step explicit EULER stepper (not recommended)
    	RK4 = 1		//! fixed time-step RK4 stepper (recommended for best accuracy to speed trade-off)
    };

    //! iLQG Settings default constructor
    /*!
     * sets all settings to default values.
     */
    iLQGSettings() :
    	integrator(RK4),
		epsilon(1e-5),
		dt(0.001),
		dt_sim(0.001),
		min_cost_improvement(1e-4), // cost needs to be at least 1e-4 better before we assume convergence
		max_iterations(100),
		fixedHessianCorrection(false),
		recordSmallestEigenvalue(false),
		nThreads(4),
		nThreadsEigen(4),
    	lineSearchSettings()
    {
    }

    INTEGRATOR integrator;	//! which integrator to use during the iLQG forward rollout
    double epsilon;			//! Eigenvalue correction factor for Hessian regularization
    double dt;				//! sampling time for the control input (seconds)
    double dt_sim;			//! sampling time for the forward simulation (seconds) \warning dt_sim needs to be an integer multiple of dt.
    double min_cost_improvement;	//! minimum cost improvement between two interations to assume convergence
    int max_iterations;		//! the maximum admissible number of iLQG main iterations \warning make sure to select this number high enough allow for convergence
    bool fixedHessianCorrection; //! perform Hessian regularization by incrementing the eigenvalues by epsilon.
    bool recordSmallestEigenvalue;	//! save the smallest eigenvalue of the Hessian
    size_t nThreads; //! number of threads, for MP version
    size_t nThreadsEigen; //! number of threads for eigen parallelization (applies both to MP and standard)
    iLQGLineSearchSettings lineSearchSettings; //! the line search settings


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


    //! print the current iLQG settings to console
    void print()
    {
        std::cout<<"iLQG Settings: "<<std::endl;
        std::cout<<"==============="<<std::endl;
        std::cout<<"integrator: "<<integratorToString[integrator]<<std::endl;
        std::cout<<"dt: "<<dt<<std::endl;
        std::cout<<"dt_sim: "<<dt_sim<<std::endl;
        std::cout<<"maxIter: "<<max_iterations<<std::endl;
        std::cout<<"fixedHessianCorrection: "<<fixedHessianCorrection<<std::endl;
        std::cout<<"recordSmallestEigenvalue: "<<recordSmallestEigenvalue<<std::endl;
        std::cout<<"epsilon: "<<epsilon<<std::endl;
        std::cout<<"nThreads: "<<nThreads<<std::endl;
        std::cout<<"nThreadsEigen: "<<nThreadsEigen<<std::endl<<std::endl;

        lineSearchSettings.print();
    }


    //! perform a quick check if the given iLQG settings fulfil minimum requirements
    /*!
     * \warning This check cannot guarantee that the control problem is well parameterized
     */
    bool parametersOk() const
    {
    	if (dt == 0 || dt_sim == 0)
		{
			std::cout << "Either iLQG or simulation timestep is zero." << std::endl;
			return false;
		}

		if (std::fmod(dt, dt_sim) > 1e-6)
		{
			std::cout << "iLQG freqency dt should be a multiple of integration frequency dt_sim." << std::endl;
			return false;
		}

		if (nThreads > 100 || nThreadsEigen > 100)
		{
			std::cout << "Number of threads should not exceed 100." << std::endl;
			return false;
		}

		return lineSearchSettings.parametersOk();
    }


    //! load iLQG Settings from file
    /*!
     *
     * @param filename path to the settings file
     * @param verbose print out settings after reading them
     * @param ns (optional) namspace of parameter file
     */
    void load(const std::string& filename, bool verbose = true, const std::string& ns = "ilqg")
    {
    	if (verbose)
    		std::cout << "Trying to load iLQG config from "<<filename<<": "<<std::endl;

		boost::property_tree::ptree pt;
		boost::property_tree::read_info(filename, pt);

		try {
			epsilon = pt.get<double>(ns +".epsilon");
		} catch (...)
		{}

		dt = pt.get<double>(ns +".dt");
		dt_sim = pt.get<double>(ns +".dt_sim");
		min_cost_improvement = pt.get<double>(ns +".min_cost_improvement");
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
			std::cout << "Loaded iLQG config from "<<filename<<": "<<std::endl;
			print();
		}

		lineSearchSettings.load(filename, verbose, ns+".line_search");
    }


    //! load settings from config file and return as settings struct
    /*!
     * @param filename the path to the settings file
     * @param verbose print settings
     * @param ns (optional) settings namespace
     * @return the newly generated settings struct
     */
    static iLQGSettings fromConfigFile(const std::string& filename, bool verbose = true, const std::string& ns = "ilqg")
    {
    	iLQGSettings settings;
    	settings.load(filename, true, ns);
    	return settings;
    }

private:
    std::map<INTEGRATOR, std::string> integratorToString = {{EULER, "Euler"}, {RK4 , "Runge-Kutta 4th order"}};
    std::map<std::string, INTEGRATOR> stringToIntegrator = {{"Euler", EULER}, {"RK4", RK4}};

};




}
}



#endif /* INCLUDE_ILQG_SETTINGS_HPP_ */
