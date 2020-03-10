/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/


#pragma once


#include <ct/optcon/nlp/Nlp.h>
#include "NlpSolver.h"

#ifdef BUILD_WITH_SNOPT_SUPPORT  // build SNOPT interface
#include <snoptProblem.hpp>
#endif  //BUILD_WITH_SNOPT_SUPPORT

namespace ct {
namespace optcon {

#ifdef BUILD_WITH_SNOPT_SUPPORT  // build SNOPT interface

/**
 * @ingroup    NLP
 *
 * @brief      Forward declaration
 */
class SnoptSolver;


/**
 * @ingroup    NLP
 *
 * @brief      Contains all the dynamically allocated memory for SNOPT
 *
 * For the implementation see ct/ct_optcon/src/nlp/solver/SnoptSolver.cpp
 *
 */
struct SnoptMemory
{
    typedef double Number;
    const SnoptSolver& self; /*!<A reference the the Snoptsolver the memory points to*/


    Number* x_ = nullptr;    /*!<Optimization variables*/
    Number* xlow_ = nullptr; /*!<Lower bound of the optimization variables*/
    Number* xupp_ = nullptr; /*!<Upper bound of the optimization variables*/
    Number* xmul_ = nullptr; /*!<The optimization variables multiplier*/
    int* xstate_ = nullptr;  //*!<The state of the optimization variables*/

    Number* F_ = nullptr;    /*!<Nonlinear parts of the costfunction and the constraints*/
    Number* Flow_ = nullptr; /*!<Lower bound on F*/
    Number* Fupp_ = nullptr; /*!<Upper bound on F*/
    Number* Fmul_ = nullptr; /*!<The F multiplier*/
    int* Fstate_ = nullptr;  /*!<The F state*/

    Number* A_ = nullptr;  /*!<Contains the linear parts of costfunction and the constraints*/
    int* iAfun_ = nullptr; /*!<Rows of the sparsity pattern of A*/
    int* jAvar_ = nullptr; /*!<Columns of the sparsity pattern of A*/

    /*!<The sparsity pattern of the costgradient and constraint jacobian*/
    int* iGfun_ = nullptr; /*!<Sparsity rows*/
    int* jGvar_ = nullptr; /*!<Sparsity columns*/

    static std::vector<SnoptMemory*> mempool; /*!<Containts all the instances of the snopt memory blocks*/
    int memind;                               /*!<The index inside the mempool this instance points to*/

    /**
     * @brief      Custom constructor
     *
     * @param[in]  self  A reference to the Snoptsolver the memory is pointing
     *                   to
     */
    SnoptMemory(const SnoptSolver& self);

    /// Destructor
    ~SnoptMemory();
};


/**
 * @ingroup    NLP
 *
 * @brief      The interface to the NLP solver SNOPT. Currently the SnoptA C++
 *             interface is implemented, which does not require the distinction
 *             between linear and non linear parts of the userfunction
 */
class SnoptSolver : public NlpSolver
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef double Number;
    typedef NlpSolver BASE;
    typedef Eigen::Matrix<Number, Eigen::Dynamic, 1> VectorXd;
    typedef Eigen::Map<VectorXd> MapVecXd;
    typedef Eigen::Map<const VectorXd> MapConstVecXd;
    typedef Eigen::Map<Eigen::VectorXi> MapVecXi;

    /**
	 * @brief      Default constructor
	 */
    SnoptSolver() {}
    /**
	 * @brief      Destructor, releases the memory
	 */
    virtual ~SnoptSolver();
    // {
    // 	free_memory(memoryPtr_);
    // }


    /**
	 * @brief      Custom constructor
	 *
	 * @param[in]  nlp       The nlp
	 * @param[in]  settings  The settings
	 */
    SnoptSolver(std::shared_ptr<Nlp> nlp, const NlpSolverSettings& settings);


    /**
	 * @brief      The non static NLP function interfacing with the optimization
	 *             module of the control toolbox
	 *
	 * @param      m       A pointer to the dynamic SNOPT memory
	 * @param      Status  Indicates the first and last call to the method
	 * @param      n       The number of optimization variables
	 * @param      x       The optimization variables
	 * @param      needF   If true, the method needs to provide f
	 * @param      neF     The length of the F vector
	 * @param      F       The F vector, containing the costfunction and the
	 *                     constraints
	 * @param      needG   If true, the method needs to provide g
	 * @param      neG     The length of the G vector
	 * @param      G       The vector of derivatives of F with respect to the
	 *                     decision variables
	 * @param      cu      The character array for external user inputs
	 * @param      lencu   The length of cu
	 * @param      iu      The integer array for external user inputs
	 * @param      leniu   The length of ui
	 * @param      ru      The real array for external user inputs
	 * @param      lenru   The length of ru
	 */
    void NLP_Function(SnoptMemory* m,
        int* Status,
        int* n,
        double x[],
        int* needF,
        int* neF,
        double F[],
        int* needG,
        int* neG,
        double G[],
        char* cu,
        int* lencu,
        int iu[],
        int* leniu,
        double ru[],
        int* lenru) const;

    /**
	 * @brief      The static NLP function passed to SNOPT
	 *
	 * @param      Status  Indicates the first and last call to the method
	 * @param      n       The number of optimization variables
	 * @param      x       The optimization variables
	 * @param      needF   If true, the method needs to provide f
	 * @param      neF     The length of the F vector
	 * @param      F       The F vector, containing the costfunction and the
	 *                     constraints
	 * @param      needG   If true, the method needs to provide g
	 * @param      neG     The length of the G vector
	 * @param      G       The vector of derivatives of F with respect to the
	 *                     decision variables
	 * @param      cu      The character array for external user inputs
	 * @param      lencu   The length of cu
	 * @param      iu      The integer array for external user inputs
	 * @param      leniu   The length of ui
	 * @param      ru      The real array for external user inputs
	 * @param      lenru   The length of ru
	 */
    static void NLP_Function(int* Status,
        int* n,
        double x[],
        int* needF,
        int* neF,
        double F[],
        int* needG,
        int* neG,
        double G[],
        char* cu,
        int* lencu,
        int iu[],
        int* leniu,
        double ru[],
        int* lenru);

    virtual void configureDerived(const NlpSolverSettings& settings) override;

    virtual bool solve() override;

    virtual void prepareWarmStart(size_t maxIterations) override;


private:
    /**
     * @brief      Allocates memory for the SNOPT interface
     *
     * @return     Pointer to the memory location
     */
    SnoptMemory* alloc_memory() const { return new SnoptMemory(*this); }
    /**
     * @brief      Frees the allocated memory
     *
     * @param[in]  mem   The memory to be freed
     */
    inline void free_memory(SnoptMemory* mem) const { delete mem; }
    /**
     * @brief      Initializes the memory
     *
     * @param      mem   The memory
     */
    void init_memory(SnoptMemory* mem) const;

    /**
     * @brief      Fills the memory with values from NLP, gets called before
     *             solving the NLP
     *
     * @param      mem   The memory
     */
    void fill_memory(SnoptMemory* mem) const;

    /**
	 * @brief      Provides SNOPT the information from SNOPT memory, gets called
	 *             before solving the NLP
	 */
    void setupSnoptObjects();

    /**
	 * @brief      Prints out status messages depending on the snopt status
	 *
	 * @param[in]  status  The SNOPT solver status
	 */
    void validateSNOPTStatus(const int& status) const;

    /**
	 * @brief      Sets the solver options.
	 */
    void setSolverOptions();

    SnoptSettings settings_;
    SnoptMemory* memoryPtr_;

    snoptProblemA snoptApp_;

    int n_ = 0;   /*!<Number of optimization variables in optimization problem*/
    int neF_ = 0; /*!<Number of constraints in optimization problem plus 1 (objective function belongs to constraints)*/
    int ObjRow_ = 0;      /*!<Objective function row*/
    Number ObjAdd_ = 0.0; /*!<Constant to be added to objective function*/

    const int Cold_ = 0, Basis_ = 1, Warm_ = 2; /*!<Defines the warmstart options for SNOPT*/
    int currStartOption_ = Cold_;               /*!<The option handed to SNOPT*/


    int lenA_ =
        0;        /*!<The dimension of the sparsity pattern for the linear part of the problem. We set it to zero to treat all parts nonlinearly*/
    int neA_ = 0; /*!<The number of non zeros in A. We set it to zero to treat all parts nonlinearly*/

    int lenG_ = 0; /*!<The dimension of the sparsity pattern for the linear part of the problem. Will be updated later*/
    int neG_ = 0;  /*!<The number of non zeros in G. Will be updated later*/

    int status_ = 0; /*!<The exit status of the SNOPT solver*/
};


#else  // BUILD_WITH_SNOPT_SUPPORT -- not building with SNOPT support, create dummy class

class SnoptSolver : public NlpSolver  // <STATE_DIM, CONTROL_DIM>
{
public:
    SnoptSolver() { throw(std::runtime_error("Error - SNOPT interface not compiled.")); }
    SnoptSolver(std::shared_ptr<Nlp> nlp, NlpSolverSettings settings)
    {
        throw(std::runtime_error("Error - SNOPT interface not compiled."));
    }

    virtual void configureDerived(const NlpSolverSettings& settings) override {}
    virtual bool solve() override { return false; }
    virtual void prepareWarmStart(size_t maxIterations) override {}
};

#endif  // BUILD_WITH_SNOPT_SUPPORT

}  // namespace optcon
}  // namespace ct
