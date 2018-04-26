/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

/*!
 * \ingroup Filter
 *
 * \brief Filter base acts like an interface for any filter type.
 *
 * @tparam OUTPUT_DIM  dimensionality of the measured output
 * @tparam STATE_DIM
 */
template <size_t OUTPUT_DIM, size_t STATE_DIM, typename SCALAR = double>
class FilterBase
{
public:
    using state_vector_t = ct::core::StateVector<STATE_DIM, SCALAR>;
    using output_vector_t = ct::core::OutputVector<OUTPUT_DIM, SCALAR>;
    using Time_t = ct::core::Time;

    //! Virtual destructor.
    virtual ~FilterBase() {}
    //! Filter method. Updates the estimate based on the received measurement and it's time stamp.
    virtual state_vector_t filter(const output_vector_t& y, const Time_t& t) = 0;
};

}  // optcon
}  // ct
