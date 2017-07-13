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


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void NLOptConSolver<STATE_DIM, CONTROL_DIM, SCALAR>::initialize(const OptConProblem_t& optConProblem, const Settings_t& settings)
{

	if(settings.nThreads > 1)
		//	nlocBackend_ = std::shared_ptr<NLOCBackendBase<STATE_DIM, CONTROL_DIM>>(new NLOCBackendMP<STATE_DIM, CONTROL_DIM>(optConProblem, settings));
		throw std::runtime_error("Selection of MP algorithms not implemented");
	else
		nlocBackend_ = std::shared_ptr<NLOCBackendBase<STATE_DIM, CONTROL_DIM>>(new NLOCBackendST<STATE_DIM, CONTROL_DIM>(optConProblem, settings));


	configure(settings);

} // configure()

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void NLOptConSolver<STATE_DIM, CONTROL_DIM, SCALAR>::configure(const Settings_t& settings)
{
	if (nlocBackend_->getSettings().nThreads != settings.nThreads)
		throw std::runtime_error("cannot switch from ST to MT or vice versa. Please call initialize.");

	switch(settings.nlocp_algorithm)
	{
	case NLOptConSettings::NLOCP_ALGORITHM::GNMS:
		nlocAlgorithm_ = std::shared_ptr<NLOCAlgorithm<STATE_DIM, CONTROL_DIM>> ( new GNMS_CT<STATE_DIM, CONTROL_DIM, SCALAR>(nlocBackend_, settings) );
		break;
	default:
		throw std::runtime_error("This algorithm is not implemented in NLOptConSolver.hpp");
	}
}

} // namespace optcon
} // namespace ct


#endif /* NLOPTCONSOLVER_IMPL_HPP_ */
