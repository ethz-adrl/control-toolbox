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

template <size_t STATE_DIM, size_t CONTROL_DIM>
ConstraintsContainerDms<STATE_DIM, CONTROL_DIM>::ConstraintsContainerDms(
		std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM>> w,
		std::shared_ptr<TimeGrid> timeGrid,
		std::vector<std::shared_ptr<ShotContainer<STATE_DIM, CONTROL_DIM>>> shotContainers,
		std::shared_ptr<ConstraintDiscretizer<STATE_DIM, CONTROL_DIM>> discretizedConstraints,
		const state_vector_t& x0,
		const DmsSettings settings) 
	:
	settings_(settings),
	shotContainers_(shotContainers)
{
	c_init_ = std::shared_ptr<InitStateConstraint<STATE_DIM, CONTROL_DIM>> (
			new InitStateConstraint<STATE_DIM, CONTROL_DIM> (x0, w));

	constraints_.push_back(c_init_);

	for(size_t shotNr = 0; shotNr < settings_.N_; shotNr++)
	{
		std::shared_ptr<ContinuityConstraint<STATE_DIM, CONTROL_DIM>> c_i = std::shared_ptr<ContinuityConstraint<STATE_DIM, CONTROL_DIM>> (
			new ContinuityConstraint<STATE_DIM, CONTROL_DIM>(shotContainers[shotNr], w, shotNr, settings));

		constraints_.push_back(c_i);
	}
	
	if(settings_.objectiveType_ == DmsSettings::OPTIMIZE_GRID)
	{
		std::shared_ptr<TimeHorizonEqualityConstraint<STATE_DIM, CONTROL_DIM>> c_horizon_equal = 
			std::shared_ptr<TimeHorizonEqualityConstraint<STATE_DIM, CONTROL_DIM>> (
				new TimeHorizonEqualityConstraint<STATE_DIM, CONTROL_DIM> (w, timeGrid, settings));

		constraints_.push_back(c_horizon_equal);
	}

	if(discretizedConstraints)
	{
		std::cout << "Adding discretized constraints" << std::endl;
		constraints_.push_back(discretizedConstraints); 
	}

}

template <size_t STATE_DIM, size_t CONTROL_DIM>
void ConstraintsContainerDms<STATE_DIM, CONTROL_DIM>::prepareEvaluation()
{
	#pragma omp parallel for num_threads( settings_.nThreads_ )
	for(auto shotContainer = shotContainers_.begin(); shotContainer < shotContainers_.end(); ++shotContainer){
		(*shotContainer)->integrateShot();
	}	
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
void ConstraintsContainerDms<STATE_DIM, CONTROL_DIM>::prepareJacobianEvaluation()
{	
	#pragma omp parallel for num_threads( settings_.nThreads_ )
	for(auto shotContainer = shotContainers_.begin(); shotContainer < shotContainers_.end(); ++shotContainer){
		(*shotContainer)->integrateShotandComputeSensitivity();
	}
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
void ConstraintsContainerDms<STATE_DIM, CONTROL_DIM>::changeInitialConstraint(const state_vector_t& x0)
{
	c_init_->updateConstraint(x0);
}
