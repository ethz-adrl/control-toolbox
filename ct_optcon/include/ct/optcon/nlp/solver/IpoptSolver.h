/**********************************************************************************************************************
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
**********************************************************************************************************************/

#pragma once

#include <ct/optcon/nlp/Nlp.h>
#include "NlpSolver.h"

#ifdef BUILD_WITH_IPOPT_SUPPORT  // build IPOPT interface

#include <cstddef>
#include <stddef.h>
#include <coin/IpTNLP.hpp>
#include <coin/IpIpoptApplication.hpp>
#include <coin/IpSolveStatistics.hpp>

#endif  // BUILD_WITH_IPOPT_SUPPORT

// #define DEBUG_PRINT

namespace ct {
namespace optcon {
namespace tpl {

#ifdef BUILD_WITH_IPOPT_SUPPORT  // build IPOPT interface

/**
 * @ingroup    NLP
 *
 * @brief      The interface to the NLP solver IPOPT
 *
 * For the implementation see ct/ct_optcon/src/nlp/solver/IpoptSolver.cpp
 */
template <typename SCALAR>
class IpoptSolver : public Ipopt::TNLP, public NlpSolver<SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BASE = NlpSolver<SCALAR>;
    using VectorXs = Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>;
    using MapVecXs = Eigen::Map<VectorXs>;
    using MapConstVecXs = Eigen::Map<const VectorXs>;

    /**
	 * @brief      Custom constructor
	 *
	 * @param[in]  nlp       The nlp
	 * @param[in]  settings  The nlp settings
	 */
    IpoptSolver(std::shared_ptr<tpl::Nlp<SCALAR>> nlp, const NlpSolverSettings& settings);

    /**
	 * @brief      Destructor
	 */
    ~IpoptSolver() override;

    IpoptSolver(const IpoptSolver&) = delete;
    IpoptSolver& operator=(const IpoptSolver&) = delete;

    bool solve() override;

    void prepareWarmStart(size_t maxIterations) override;

    void configureDerived(const NlpSolverSettings& settings) override;

    /**@name Overloaded from TNLP */
    //@{
    /** Method to return some info about the nlp */
    bool get_nlp_info(Ipopt::Index& n,
        Ipopt::Index& m,
        Ipopt::Index& nnz_jac_g,
        Ipopt::Index& nnz_h_lag,
        IndexStyleEnum& index_style) override;

    /** Method to return the bounds for my problem */
    bool get_bounds_info(Ipopt::Index n, SCALAR* x_l, SCALAR* x_u, Ipopt::Index m, SCALAR* g_l, SCALAR* g_u) override;

    /** Method to return the starting point for the algorithm */
    bool get_starting_point(Ipopt::Index n,
        bool init_x,
        SCALAR* x,
        bool init_z,
        SCALAR* z_L,
        SCALAR* z_U,
        Ipopt::Index m,
        bool init_lambda,
        SCALAR* lambda) override;

    /** Method to return the objective value */
    bool eval_f(Ipopt::Index n, const SCALAR* x, bool new_x, SCALAR& obj_value) override;

    /** Method to return the gradient of the objective */
    bool eval_grad_f(Ipopt::Index n, const SCALAR* x, bool new_x, SCALAR* grad_f) override;

    /** Method to return the constraint residuals */
    bool eval_g(Ipopt::Index n, const SCALAR* x, bool new_x, Ipopt::Index m, SCALAR* g) override;

    /** Method to return:
	 *   1) The structure of the jacobian (if "values" is NULL)
	 *   2) The values of the jacobian (if "values" is not NULL)
	 */
    bool eval_jac_g(Ipopt::Index n,
        const SCALAR* x,
        bool new_x,
        Ipopt::Index m,
        Ipopt::Index nele_jac,
        Ipopt::Index* iRow,
        Ipopt::Index* jCol,
        SCALAR* values) override;

    /** Method to return:
	 *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
	 *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
	 */
    bool eval_h(Ipopt::Index n,
        const SCALAR* x,
        bool new_x,
        SCALAR obj_factor,
        Ipopt::Index m,
        const SCALAR* lambda,
        bool new_lambda,
        Ipopt::Index nele_hess,
        Ipopt::Index* iRow,
        Ipopt::Index* jCol,
        SCALAR* values) override;

    //@}

    /** @name Solution Methods */
    //@{
    /** This method is called when the algorithm is complete so the TNLP can store/write the solution */
    void finalize_solution(Ipopt::SolverReturn status,
        Ipopt::Index n,
        const SCALAR* x,
        const SCALAR* z_L,
        const SCALAR* z_U,
        Ipopt::Index m,
        const SCALAR* g,
        const SCALAR* lambda,
        SCALAR obj_value,
        const Ipopt::IpoptData* ip_data,
        Ipopt::IpoptCalculatedQuantities* ip_cq) override;
    //@}

private:
    /**
	 * @brief      Sets the IPOPT solver options.
	 */
    void setSolverOptions();
    std::shared_ptr<Ipopt::IpoptApplication> ipoptApp_; /*!< A pointer to ipopt*/
    Ipopt::ApplicationReturnStatus status_;             /*!< The return status of IPOPT*/
    IpoptSettings settings_;                            /*!< Contains the IPOPT settings*/
};

#include "implementation/IpoptSolver-impl.h"

#else  // BUILD_WITH_IPOPT_SUPPORT -- not building with IPOPT support, create dummy class

template <typename SCALAR>
class IpoptSolver : public NlpSolver<SCALAR>
{
public:
    IpoptSolver() { throw(std::runtime_error("Error - IPOPT interface not compiled.")); }
    IpoptSolver(std::shared_ptr<tpl::Nlp<SCALAR>> nlp, NlpSolverSettings settings)
    {
        throw(std::runtime_error("Error - IPOPT interface not compiled."));
    }

    bool solve() override { return false; }
    void prepareWarmStart(size_t maxIterations) override {}
    void configureDerived(const NlpSolverSettings& settings) override {}
};

#endif  // BUILD_WITH_IPOPT_SUPPORT
        //
}  // namespace tpl

using IpoptSolver = tpl::IpoptSolver<double>;

}  // namespace optcon
}  // namespace ct
