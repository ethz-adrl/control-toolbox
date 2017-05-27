#ifndef CT_OPTCON_IPOPTSOLVER_H
#define CT_OPTCON_IPOPTSOLVER_H

#include <ct/optcon/nlp/Nlp.h>
#include "NlpSolver.h"

#ifdef BUILD_WITH_IPOPT_SUPPORT 	// build IPOPT interface

#include "IpTNLP.hpp"
#include "IpIpoptApplication.hpp"
#include "IpSolveStatistics.hpp"
#include <ct/optcon/dms/util/MatlabInterface.hpp>

#endif // BUILD_WITH_IPOPT_SUPPORT

// #define DEBUG_PRINT

namespace ct {
namespace optcon {

#ifdef BUILD_WITH_IPOPT_SUPPORT 	// build IPOPT interface

class IpoptSolver : public Ipopt::TNLP, public NlpSolver
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	typedef NlpSolver BASE;
	typedef double Number;
	typedef Eigen::Matrix<Number, Eigen::Dynamic, 1> VectorXd;
	typedef Eigen::Map<VectorXd> MapVecXd;
	typedef Eigen::Map<const VectorXd> MapConstVecXd;

	/** default constructor */
	IpoptSolver(std::shared_ptr<Nlp> nlp, const NlpSolverSettings& settings);

	/** default destructor */
	virtual ~IpoptSolver();

	// Inherited from DmsSolverBase
	virtual bool solve() override;
	virtual void prepareWarmStart(size_t maxIterations) override;
	//maybe we can get rid of this method as well
	virtual void updateInitGuess() override {};
	virtual bool solveSucceeded() override;
	virtual void configureDerived(const NlpSolverSettings& settings) override;

	// Inherited from TNLP
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

	/*virtual bool intermediate_callback(AlgorithmMode mode,
                                   Ipopt::Index iter, Number obj_value,
                                   Number inf_pr, Number inf_du,
                                   Number mu, Number d_norm,
                                   Number regularization_size,
                                   Number alpha_du, Number alpha_pr,
                                   Ipopt::Index ls_trials,
                                   const IpoptData* ip_data,
                                   IpoptCalculatedQuantities* ip_cq); */

private:
	virtual void setSolverOptions() override;
	std::shared_ptr<Ipopt::IpoptApplication> ipoptApp_;
	Ipopt::ApplicationReturnStatus status_;
	IpoptSettings settings_;

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

#include "implementation/IpoptSolver-impl.h"

#else	// BUILD_WITH_IPOPT_SUPPORT -- not building with IPOPT support, create dummy class

class IpoptSolver : public NlpSolver
{
public:
	IpoptSolver(){ throw (std::runtime_error("Error - IPOPT interface not compiled.")); }
	IpoptSolver(std::shared_ptr<Nlp> nlp, NlpSolverSettings settings){
		throw (std::runtime_error("Error - IPOPT interface not compiled."));}

	virtual bool solve() override {return false;}
	virtual void prepareWarmStart(size_t maxIterations) override{}
	//maybe we can get rid of this method as well
	virtual void updateInitGuess() override {};
	virtual bool solveSucceeded() override {return false;}
	virtual void configureDerived(const NlpSolverSettings& settings) override{}
private:
	virtual void setSolverOptions() override{}
};

#endif // BUILD_WITH_IPOPT_SUPPORT

} // namespace optcon
} // namespace ct
#endif // CT_OPTCON_IPOPTSOLVER_H
