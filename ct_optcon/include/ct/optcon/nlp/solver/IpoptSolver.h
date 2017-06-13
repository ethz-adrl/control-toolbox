/***********************************************************************************
Copyright (c) 2016, Agile & Dexterous Robotics Lab, ETH ZURICH. All rights reserved.

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

#ifndef CT_OPTCON_NLP_SOLVER_IPOPTSOLVER_H_
#define CT_OPTCON_NLP_SOLVER_IPOPTSOLVER_H_

#include <ct/optcon/nlp/Nlp.h>
#include "NlpSolver.h"

#ifdef BUILD_WITH_IPOPT_SUPPORT 	// build IPOPT interface

#include "IpTNLP.hpp"
#include "IpIpoptApplication.hpp"
#include "IpSolveStatistics.hpp"

#endif // BUILD_WITH_IPOPT_SUPPORT

// #define DEBUG_PRINT

namespace ct {
namespace optcon {

#ifdef BUILD_WITH_IPOPT_SUPPORT 	// build IPOPT interface

/**
 * @ingroup    NLP
 *
 * @brief      The interface to the NLP solver IPOPT
 *
 * For the implementation see ct/ct_optcon/src/nlp/solver/IpoptSolver.cpp
 */
class IpoptSolver : public Ipopt::TNLP, public NlpSolver
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	typedef NlpSolver BASE;
	typedef double Number;
	typedef Eigen::Matrix<Number, Eigen::Dynamic, 1> VectorXd;
	typedef Eigen::Map<VectorXd> MapVecXd;
	typedef Eigen::Map<const VectorXd> MapConstVecXd;

	/**
	 * @brief      Custom constructor
	 *
	 * @param[in]  nlp       The nlp
	 * @param[in]  settings  The nlp settings
	 */
	IpoptSolver(std::shared_ptr<Nlp> nlp, const NlpSolverSettings& settings);

	/**
	 * @brief      Destructor
	 */
	virtual ~IpoptSolver();

	virtual bool solve() override;

	virtual void prepareWarmStart(size_t maxIterations) override;

	virtual void configureDerived(const NlpSolverSettings& settings) override;

	/**@name Overloaded from TNLP */
	//@{
	/** Method to return some info about the nlp */
	virtual bool get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
			Ipopt::Index& nnz_h_lag, IndexStyleEnum& index_style);

	/** Method to return the bounds for my problem */
	virtual bool get_bounds_info(Ipopt::Index n, Number* x_l, Number* x_u,
			Ipopt::Index m, Number* g_l, Number* g_u);

	/** Method to return the starting point for the algorithm */
	virtual bool get_starting_point(Ipopt::Index n, bool init_x, Number* x,
			bool init_z, Number* z_L, Number* z_U,
			Ipopt::Index m, bool init_lambda,
			Number* lambda);

	/** Method to return the objective value */
	virtual bool eval_f(Ipopt::Index n, const Number* x, bool new_x, Number& obj_value);

	/** Method to return the gradient of the objective */
	virtual bool eval_grad_f(Ipopt::Index n, const Number* x, bool new_x, Number* grad_f);

	/** Method to return the constraint residuals */
	virtual bool eval_g(Ipopt::Index n, const Number* x, bool new_x, Ipopt::Index m, Number* g);

	/** Method to return:
	 *   1) The structure of the jacobian (if "values" is NULL)
	 *   2) The values of the jacobian (if "values" is not NULL)
	 */
	virtual bool eval_jac_g(Ipopt::Index n, const Number* x, bool new_x,
			Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow, Ipopt::Index *jCol,
			Number* values);

	/** Method to return:
	 *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
	 *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
	 */
	virtual bool eval_h(Ipopt::Index n, const Number* x, bool new_x,
			Number obj_factor, Ipopt::Index m, const Number* lambda,
			bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow,
			Ipopt::Index* jCol, Number* values);

	//@}

	/** @name Solution Methods */
	//@{
	/** This method is called when the algorithm is complete so the TNLP can store/write the solution */
	virtual void finalize_solution(Ipopt::SolverReturn status,
			Ipopt::Index n, const Number* x, const Number* z_L, const Number* z_U,
			Ipopt::Index m, const Number* g, const Number* lambda,
			Number obj_value,
			const Ipopt::IpoptData* ip_data,
			Ipopt::IpoptCalculatedQuantities* ip_cq);
	//@}

private:

	/**
	 * @brief      Sets the IPOPT solver options.
	 */
	void setSolverOptions();
	std::shared_ptr<Ipopt::IpoptApplication> ipoptApp_;  /*!< A pointer to ipopt*/
	Ipopt::ApplicationReturnStatus status_; /*!< The return status of IPOPT*/
	IpoptSettings settings_; /*!< Contains the IPOPT settings*/

	/**@name Methods to block default compiler methods.
	 * The compiler automatically generates the following three methods.
	 *  Since the default compiler implementation is generally not what
	 *  you want (for all but the most simple classes), we usually
	 *  put the declarations of these methods in the private section
	 *  and never implement them. This prevents the compiler from
	 *  implementing an incorrect "default" behavior without us
	 *  knowing. (See Scott Meyers book, "Effective C++")
	 *
	 */
	//@{
	IpoptSolver(const IpoptSolver&);
	IpoptSolver& operator=(const IpoptSolver&);
	//@}

};

#else	// BUILD_WITH_IPOPT_SUPPORT -- not building with IPOPT support, create dummy class

class IpoptSolver : public NlpSolver
{
public:
	IpoptSolver(){ throw (std::runtime_error("Error - IPOPT interface not compiled.")); }
	IpoptSolver(std::shared_ptr<Nlp> nlp, NlpSolverSettings settings){
		throw (std::runtime_error("Error - IPOPT interface not compiled."));}

	virtual bool solve() override {return false;}
	virtual void prepareWarmStart(size_t maxIterations) override{}
	virtual void configureDerived(const NlpSolverSettings& settings) override{}
};

#endif // BUILD_WITH_IPOPT_SUPPORT

} // namespace optcon
} // namespace ct

#endif // CT_OPTCON_NLP_SOLVER_IPOPTSOLVER_H_
