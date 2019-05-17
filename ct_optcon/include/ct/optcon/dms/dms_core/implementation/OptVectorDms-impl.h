/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>::OptVectorDms(size_t n, const DmsSettings& settings)
    : tpl::OptVector<SCALAR>(n), settings_(settings), numPairs_(settings.N_ + 1)
{
    size_t currIndex = 0;

    for (size_t i = 0; i < numPairs_; i++)
    {
        pairNumToStateIdx_.insert(std::make_pair(i, currIndex));
        currIndex += STATE_DIM;

        pairNumToControlIdx_.insert(std::make_pair(i, currIndex));
        currIndex += CONTROL_DIM;
    }
    stateSolution_.resize(numPairs_);
    inputSolution_.resize(numPairs_);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename DmsDimensions<STATE_DIM, CONTROL_DIM, SCALAR>::state_vector_t
OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>::getOptimizedState(const size_t pairNum) const
{
    size_t index = getStateIndex(pairNum);
    return (this->x_.segment(index, STATE_DIM));
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename DmsDimensions<STATE_DIM, CONTROL_DIM, SCALAR>::control_vector_t
OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>::getOptimizedControl(const size_t pairNum) const
{
    size_t index = getControlIndex(pairNum);
    return (this->x_.segment(index, CONTROL_DIM));
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
const typename DmsDimensions<STATE_DIM, CONTROL_DIM, SCALAR>::state_vector_array_t&
OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>::getOptimizedStates()
{
    for (size_t i = 0; i < numPairs_; i++)
        stateSolution_[i] = getOptimizedState(i);

    return stateSolution_;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
const typename DmsDimensions<STATE_DIM, CONTROL_DIM, SCALAR>::control_vector_array_t&
OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>::getOptimizedInputs()
{
    for (size_t i = 0; i < numPairs_; i++)
        inputSolution_[i] = getOptimizedControl(i);

    return inputSolution_;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>::getStateIndex(const size_t pairNum) const
{
    return pairNumToStateIdx_.find(pairNum)->second;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>::getControlIndex(const size_t pairNum) const
{
    return pairNumToControlIdx_.find(pairNum)->second;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>::changeInitialState(const state_vector_t& x0)
{
    size_t s_index = getStateIndex(0);
    this->x_.segment(s_index, STATE_DIM) = x0;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>::changeDesiredState(const state_vector_t& xF)
{
    size_t s_index = pairNumToStateIdx_.find(settings_.N_)->second;
    this->x_.segment(s_index, STATE_DIM) = xF;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>::setInitGuess(const state_vector_t& x0,
    const state_vector_t& x_f,
    const control_vector_t& u0)
{
    size_t type = 1;  // 0 = constant init guess, 1 = linearly interpolated between x0 and x_f

    // init the states s_i and controls q_i
    for (size_t i = 0; i < numPairs_; i++)
    {
        size_t s_index = getStateIndex(i);
        size_t q_index = getControlIndex(i);

        switch (type)
        {
            case 0:
            {
                this->xInit_.segment(s_index, STATE_DIM) = x0;
                break;
            }
            case 1:
                this->xInit_.segment(s_index, STATE_DIM) = x0 + (x_f - x0) * (i / (numPairs_ - 1));
                break;
        }
        this->xInit_.segment(q_index, CONTROL_DIM) = u0;
    }
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>::setInitGuess(const state_vector_array_t& x_init,
    const control_vector_array_t& u_init)
{
    if (x_init.size() != numPairs_)
        throw std::runtime_error("initial guess state trajectory not matching number of shots");
    if (u_init.size() != numPairs_)
        throw std::runtime_error("initial guess input trajectory not matching number of shots");

    for (size_t i = 0; i < numPairs_; i++)
    {
        size_t s_index = getStateIndex(i);
        size_t q_index = getControlIndex(i);

        this->xInit_.segment(s_index, STATE_DIM) = x_init[i];
        this->xInit_.segment(q_index, CONTROL_DIM) = u_init[i];
    }
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>::printoutSolution()
{
    std::cout << "... printing solutions: " << std::endl;
    std::cout << "x_solution" << std::endl;
    state_vector_array_t x_sol = getOptimizedStates();
    for (size_t i = 0; i < x_sol.size(); ++i)
    {
        std::cout << x_sol[i].transpose() << std::endl;
    }

    std::cout << "u_solution" << std::endl;
    control_vector_array_t u_sol = getOptimizedInputs();
    for (size_t i = 0; i < u_sol.size(); ++i)
    {
        std::cout << u_sol[i].transpose() << std::endl;
    }

    std::cout << std::endl;
    std::cout << " ... done." << std::endl;
}

}  // namespace optcon
}  // namespace ct
