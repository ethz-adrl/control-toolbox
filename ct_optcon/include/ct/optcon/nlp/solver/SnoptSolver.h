#ifndef CT_OPTCON_SNOPTSOLVER_H
#define CT_OPTCON_SNOPTSOLVER_H


#include <ct/optcon/nlp/Nlp.h>
#include "NlpSolver.h"

#ifdef BUILD_WITH_SNOPT_SUPPORT 	// build SNOPT interface
#include <snoptProblem.hpp>
#endif //BUILD_WITH_SNOPT_SUPPORT

namespace ct {
namespace optcon {

#ifdef BUILD_WITH_SNOPT_SUPPORT 	// build SNOPT interface

class SnoptSolver;

struct SnoptMemory{
	typedef double Number;
    const SnoptSolver& self;


	Number *x_ 		= nullptr; 	// Primal vars
	Number *xlow_	= nullptr;	// Primal bounds
	Number *xupp_	= nullptr;
	Number *xmul_	= nullptr;	// Dual vars
	int *xstate_	= nullptr; 	// Primal State

	Number *F_    	= nullptr;		//On exit, final value of vector functions
	Number *Flow_ 	= nullptr;
	Number *Fupp_ 	= nullptr;
	Number *Fmul_ 	= nullptr;		//On exit, vector of dual variables
	int *Fstate_	= nullptr;

	int *iAfun_ 	= nullptr;
	int *jAvar_ 	= nullptr;
	Number *A_  	= nullptr;
	int *iGfun_		= nullptr;	//Hold row indices of non zero jacobian values
	int *jGvar_		= nullptr;	//Hold column indices of non zero jacoabian values

    static std::vector<SnoptMemory*> mempool;
    int memind;

    SnoptMemory(const SnoptSolver& self);

    /// Destructor
    ~SnoptMemory();

};

//template tbd
class SnoptSolver : public NlpSolver //template tbd
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	typedef double Number;
	typedef NlpSolver BASE;
	typedef Eigen::Matrix<Number, Eigen::Dynamic, 1> VectorXd;
	typedef Eigen::Map<VectorXd> MapVecXd;
	typedef Eigen::Map<const VectorXd> MapConstVecXd;
	typedef Eigen::Map<Eigen::VectorXi> MapVecXi;


	SnoptSolver(std::shared_ptr<Nlp> nlp, const NlpSolverSettings& settings);

	// static void snoptCuserfun(	int *mode,  	int *nnObj, 	int *nnCon,
	// 	   						int *nnJac, 	int *nnL,   	int *negCon, double x[],
	// 	   						double *fObj,  	double gObj[],
	// 	   						double fCon[], 	double gCon[], 	int *Status,
	//    							char    *cu, 	int *lencu,
	// 	   						int    iu[], 	int *leniu,
	// 	   						double ru[], 	int *lenru );

	void NLP_Function(SnoptMemory* m,		int    *Status, int *n,    double x[],
					        int    *needF,  int *neF,  double F[],
					        int    *needG,  int *neG,  double G[],
					        char      *cu,  int *lencu,
					        int    iu[],    int *leniu,
					        double ru[],    int *lenru ) const;

	static void NLP_Function(int    *Status, int *n,    double x[],
					         int    *needF,  int *neF,  double F[],
					         int    *needG,  int *neG,  double G[],
					         char      *cu,  int *lencu,
					         int    iu[],    int *leniu,
					         double ru[],    int *lenru );

	virtual void configureDerived(const NlpSolverSettings& settings) override;
	virtual bool solve() override;
	virtual void prepareWarmStart(size_t maxIterations) override;
	virtual void updateInitGuess() override;
	virtual bool solveSucceeded() override;

    SnoptMemory* alloc_memory() const { return new SnoptMemory(*this);}

    void free_memory(SnoptMemory *mem) const { delete mem;}

    void init_memory(SnoptMemory* mem) const;

    void fill_memory(SnoptMemory* mem) const;

	void setupSnoptObjects();

	SnoptSolver(){}

	virtual ~SnoptSolver()
	{
		free_memory(memoryPtr_);
	}

private:
	SnoptSettings settings_;
	SnoptMemory* memoryPtr_;

	void validateSNOPTStatus(const int & status) const;
	virtual void setSolverOptions() override;

	snoptProblemA snoptApp_;

	  // Allocate and initialize;
	int n_ = 0;				// Number of variables in optimization problem
	int neF_ = 0;			// Number of constraints in optimization problem plus 1 (objective function belongs to constraints)
	int    ObjRow_  = 0;	// In which row is the objective function
	Number ObjAdd_  = 0.0;	// Constant to be added to objective function

// If warm, a valid x_start, F_start, xstate, Fstate needs to be provided
	const int Cold_ = 0, Basis_ = 1, Warm_ = 2;
	int currStartOption_ = 0;

// Linear constraints
	int lenA_ = 0;
	int neA_ = 0;
// Non linear constraints
	int lenG_ = 0  ;	//Length of iGfun, jGvar
	int neG_ = 0; // neA and neG must be defined when providing derivatives
	int status_ = 0;

};


#include "implementation/SnoptSolver-impl.h"

#else	// BUILD_WITH_SNOPT_SUPPORT -- not building with SNOPT support, create dummy class

class SnoptSolver : public NlpSolver// <STATE_DIM, CONTROL_DIM>
{
public:
	SnoptSolver(){ throw (std::runtime_error("Error - SNOPT interface not compiled.")); }
	SnoptSolver(std::shared_ptr<Nlp> nlp, NlpSolverSettings settings){throw (std::runtime_error("Error - SNOPT interface not compiled."));}
	
	virtual void configureDerived(const NlpSolverSettings& settings) override{}
	virtual bool solve() override {return false;}
	virtual void prepareWarmStart(size_t maxIterations) override{}
	virtual void updateInitGuess() override{}
	virtual bool solveSucceeded() override {return false;}
private:
	virtual void setSolverOptions() override{}
};

#endif	// BUILD_WITH_SNOPT_SUPPORT

} // namespace optcon
} // namespace ct
#endif
