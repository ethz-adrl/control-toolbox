/*
 * NLOptConSolver-impl.hpp
 *
 * Created on: 13.07.2017
 * 	   Author: mgiftthaler<mgiftthaler@ethz.ch> 
 * 
 */

#ifndef NLOPTCONSOLVER_IMPL_HPP_
#define NLOPTCONSOLVER_IMPL_HPP_


namespace ct{
namespace optcon{


template <typename DERIVED, typename POLICY, typename SETTINGS, size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void NLOptConSolver<DERIVED, POLICY, SETTINGS, STATE_DIM, CONTROL_DIM, SCALAR>::configure(const Settings_t& settings){

	if(settings.nThreads > 1)
		//	nlocBackend_ = std::shared_ptr<NLOCBackendBase<STATE_DIM, CONTROL_DIM>>(new NLOCBackendMP<STATE_DIM, CONTROL_DIM>(optConProblem, settings));
		throw std::runtime_error("Selection of MP algorithms not implemented");
	else
		nlocBackend_ = std::shared_ptr<NLOCBackendBase<STATE_DIM, CONTROL_DIM>>(new NLOCBackendST<STATE_DIM, CONTROL_DIM>(optConProblem, settings));


	switch(settings.algorithm)
	{
	case GNMS_CT:
		nlocAlgorithm_ = std::shared_ptr<NLOCAlgorithm<STATE_DIM, CONTROL_DIM>> ( new GNMS_CT<STATE_DIM, CONTROL_DIM, SCALAR>(nlocBackend_) );
		break;
	default:
		throw std::runtime_error("This algorithm is not implemented in NLOptConSolver.hpp");
	}

} // configure()

} // namespace optcon
} // namespace ct


#endif /* NLOPTCONSOLVER_IMPL_HPP_ */
