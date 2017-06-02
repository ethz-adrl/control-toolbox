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

#ifndef CT_OPTCON_DMS_CORE_SHOTINTEGRATORBASE_H_
#define CT_OPTCON_DMS_CORE_SHOTINTEGRATORBASE_H_


#include <ct/core/core.h>

#include <ct/optcon/dms/dms_core/DmsDimensions.h>

namespace ct{
namespace optcon{

/**
 * @brief      The abstract base class used for the shotintegrators
 *
 * @tparam     STATE_DIM    The state dimension
 * @tparam     CONTROL_DIM  The control dimension
 */
template<size_t STATE_DIM, size_t CONTROL_DIM>
class ShotIntegratorBase {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef DmsDimensions<STATE_DIM, CONTROL_DIM> DIMENSIONS;

	typedef typename DIMENSIONS::state_vector_t  state_vector_t;
	typedef typename DIMENSIONS::control_vector_t control_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
	typedef typename DIMENSIONS::control_vector_array_t control_vector_array_t;
	typedef typename DIMENSIONS::time_array_t time_array_t;

	typedef typename DIMENSIONS::state_matrix_t state_matrix_t;
	typedef typename DIMENSIONS::control_matrix_t control_matrix_t;
	typedef typename DIMENSIONS::state_control_matrix_t state_control_matrix_t;
	typedef typename DIMENSIONS::state_matrix_array_t state_matrix_array_t;
	typedef typename DIMENSIONS::state_control_matrix_array_t state_control_matrix_array_t;

	/**
	 * @brief      Default constructor
	 */
	ShotIntegratorBase(){}

	/**
	 * @brief      The destructor
	 */
	virtual ~ShotIntegratorBase(){}

	/**
	 * @brief      Sets up the system before integration
	 */
	virtual void setupSystem() = 0;

	/**
	 * @brief      Performs the integration using the ct core integrators
	 */
	virtual void integrate() = 0;

	/**
	 * @brief      Retrieves the trajectories obtained by the integration
	 *
	 * @param      timeTraj   The time trajectory
	 * @param      stateTraj  The state trajectory
	 * @param      cost       The cost trajectory
	 */
	virtual void retrieveStateTrajectories(
			time_array_t& timeTraj,
			state_vector_array_t& stateTraj,
			double& cost
	) = 0;

	/**
	 * @brief      Retrieves the sensitivity trajectories obtained by the integration
	 *
	 * @param      timeTraj          The time trajectory
	 * @param      stateTraj         The state trajectory
	 * @param      dXdSiTraj         The ODE senitivity wrt s_i
	 * @param      dXdQiTraj         The ODE senitivity wrt q_i
	 * @param      dXdQip1Traj       The ODE senitivity wrt q_{i+1}
	 * @param      dXdHiTraj         The ODE senitivity wrt h_i
	 * @param      costGradientSi    The cost gradient wrt s_i
	 * @param      costGradientQi    The cost gradient wrt q_i
	 * @param      costGradientQip1  The cost gradient wrt q_{i+1}
	 * @param      costGradientHi    The cost gradient wrt h_i
	 */
	virtual void retrieveTrajectories(
			time_array_t& timeTraj,
			state_vector_array_t& stateTraj,
			state_matrix_array_t& dXdSiTraj,
			state_control_matrix_array_t& dXdQiTraj,
			state_control_matrix_array_t& dXdQip1Traj,
			state_vector_array_t& dXdHiTraj,
			state_vector_t& costGradientSi,
			control_vector_t& costGradientQi,
			control_vector_t& costGradientQip1,
			double& costGradientHi
	) = 0;


};

}//namespace optcon
}//namespace ct


#endif /* SHOTINTEGRATORBASE_HPP_ */
