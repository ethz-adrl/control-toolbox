#ifndef CT_NLPSOLVER_HPP
#define CT_NLPSOLVER_HPP

#include <ct/optcon/nlp/Nlp.h>
#include "NlpSolverSettings.h"

namespace ct {
namespace optcon {

class NlpSolver
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	NlpSolver() :
	isInitialized_(false)
	{}

	NlpSolver(std::shared_ptr<Nlp> nlp, NlpSolverSettings settings) 
	: 
		nlp_(nlp),
		settings_(settings),
		isInitialized_(false)
	{}

	NlpSolver(std::shared_ptr<Nlp> nlp) 
	: 
		nlp_(nlp)
	{}

	virtual ~NlpSolver() {}

	void configure(const NlpSolverSettings& settings) 
	{
		settings_ = settings;
		configureDerived(settings);
	} 

	virtual void configureDerived(const NlpSolverSettings& settings) = 0;
	virtual bool solve() = 0;
	virtual void prepareWarmStart(size_t maxIterations) = 0;
	virtual void updateInitGuess() = 0;
	virtual bool solveSucceeded() = 0;

protected:
	virtual void setSolverOptions() = 0;
	std::shared_ptr<Nlp> nlp_;
	NlpSolverSettings settings_;
	bool isInitialized_;
};

} // namespace ct
} // namespace optcon



#endif //CT_NLPSOLVER_HPP