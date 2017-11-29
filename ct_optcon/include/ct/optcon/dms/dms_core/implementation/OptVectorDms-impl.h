/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>::OptVectorDms(size_t n, const DmsSettings& settings)
    : tpl::OptVector<SCALAR>(n), settings_(settings), numPairs_(settings.N_ + 1)
{
    // if (settings_.objectiveType_ == DmsSettings::OPTIMIZE_GRID)
    // 	optimizedTimeSegments_.resize(numPairs_ - 1);

    size_t currIndex = 0;

    for (size_t i = 0; i < numPairs_; i++)
    {
        pairNumToStateIdx_.insert(std::make_pair(i, currIndex));
        currIndex += STATE_DIM;

        pairNumToControlIdx_.insert(std::make_pair(i, currIndex));
        currIndex += CONTROL_DIM;

        // if (settings_.objectiveType_ == DmsSettings::OPTIMIZE_GRID)
        // 	if(i < numPairs_-1)
        // 	{
        // 		shotNumToShotDurationIdx_.insert(std::make_pair(i, currIndex));
        // 		currIndex += 1;
        // 	}
    }

    // if (settings_.objectiveType_ == DmsSettings::OPTIMIZE_GRID)
    // 	setLowerTimeSegmentBounds();
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
SCALAR OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>::getOptimizedTimeSegment(const size_t pairNum) const
{
    size_t index = getTimeSegmentIndex(pairNum);
    // std::cout << "x_(index) : " << x_(index) << std::endl;
    return this->x_(index);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
const typename DmsDimensions<STATE_DIM, CONTROL_DIM, SCALAR>::state_vector_array_t&
OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>::getOptimizedStates()
{
    stateSolution_.clear();
    for (size_t i = 0; i < numPairs_; i++)
    {
        stateSolution_.push_back(getOptimizedState(i));
    }
    return stateSolution_;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
const typename DmsDimensions<STATE_DIM, CONTROL_DIM, SCALAR>::control_vector_array_t&
OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>::getOptimizedInputs()
{
    inputSolution_.clear();
    for (size_t i = 0; i < numPairs_; i++)
        inputSolution_.push_back(getOptimizedControl(i));

    return inputSolution_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
const Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>& OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>::getOptimizedTimeSegments()
{
    for (size_t i = 0; i < numPairs_ - 1; ++i)
        optimizedTimeSegments_(i) = getOptimizedTimeSegment(i);

    return optimizedTimeSegments_;
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
size_t OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>::getTimeSegmentIndex(const size_t shotNr) const
{
    return (shotNumToShotDurationIdx_.find(shotNr)->second);
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

    // if(settings_.objectiveType_ == DmsSettings::OPTIMIZE_GRID)
    // {
    // 	for (size_t i = 0; i< numPairs_ - 1; i++)
    // 	{
    // 		size_t h_index = getTimeSegmentIndex(i);
    // 		x_(h_index) = (double)settings_.T_ / (double)settings_.N_;
    // 	}
    // }
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

    // if(settings_.objectiveType_ == DmsSettings::OPTIMIZE_GRID)
    // {
    // 	for (size_t i = 0; i< numPairs_ - 1; i++)
    // 	{
    // 		size_t h_index = getTimeSegmentIndex(i);
    // 		x_(h_index) = (double)settings_.T_ / (double)settings_.N_;
    // 	}
    // }

    // std::cout << "x_ init: " << x_.transpose() << std::endl;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>::setLowerTimeSegmentBounds()
{
    for (size_t i = 0; i < numPairs_ - 1; i++)
    {
        size_t h_index = shotNumToShotDurationIdx_.find(i)->second;
        Eigen::Matrix<SCALAR, 1, 1> newElement;
        newElement(0, 0) = settings_.h_min_;
        this->xLb_.segment(h_index, 1) = newElement;  //0.0;
        newElement(0, 0) = 0.5;
        this->xUb_.segment(h_index, 1) = newElement;
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

    std::cout << "t_solution" << std::endl;
    const Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>& t_sol = getOptimizedTimeSegments();
    for (size_t i = 0; i < t_sol.size(); ++i)
    {
        std::cout << t_sol[i] << "  ";
    }
    std::cout << std::endl;
    std::cout << " ... done." << std::endl;
}

}  // namespace optcon
}  // namespace ct
