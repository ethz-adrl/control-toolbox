/*
 * NLOCAlgorithm.hpp
 *
 * Created on: 13.07.2017
 * 	   Author: mgiftthaler<mgiftthaler@ethz.ch> 
 * 
 */

#ifndef NLOCALGORITHM_HPP_
#define NLOCALGORITHM_HPP_


namespace ct{
namespace optcon{


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class NLOCAlgorithm
{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef NLOCBackendBase<STATE_DIM, CONTROL_DIM, SCALAR> Backend_t;

	typedef ct::core::ConstantTrajectoryController<STATE_DIM, CONTROL_DIM, SCALAR> Policy_t;
	typedef NLOptConSettings Settings_t;
	typedef SCALAR Scalar_t;

	NLOCAlgorithm(const std::shared_ptr<Backend_t>& backend) :
		backend_(backend)
	{	}

	virtual ~NLOCAlgorithm(){}

	virtual void configure(const Settings_t& settings)  = 0;

	virtual void prepareIteration() override  = 0;

	virtual bool finishIteration() override = 0;

	virtual bool runIteration() override = 0;

	virtual void setInitialGuess(const Policy_t& initialGuess) = 0;

protected:
	std::shared_ptr<Backend_t> backend_;

};

}
}



#endif /* NLOCALGORITHM_HPP_ */
