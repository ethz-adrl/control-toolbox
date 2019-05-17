/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "ct/optcon/dms/dms_core/spline/SplinerBase.h"
#include <ct/optcon/dms/dms_core/TimeGrid.h>

namespace ct {
namespace optcon {

/**
 * @ingroup    DMS
 *
 * @brief      The linear spline implementation
 *
 * @tparam     T     The vector type to be splined
 */
template <class T, typename SCALAR = double>
class LinearSpliner : public SplinerBase<T, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef T vector_t;
    typedef std::vector<vector_t, Eigen::aligned_allocator<vector_t>> vector_array_t;
    typedef Eigen::Matrix<SCALAR, T::DIM, T::DIM> matrix_t;

    LinearSpliner() = delete;

    /**
	 * @brief      Custom constructor
	 *
	 * @param[in]  grid  The dms timegrid
	 */
    LinearSpliner(std::shared_ptr<tpl::TimeGrid<SCALAR>> grid) : timeGrid_(grid) {}
    ~LinearSpliner() override = default;
    void computeSpline(const vector_array_t& points) override { nodes_ = points; }
    // evaluate spline and return vector at interpolation time
    vector_t evalSpline(const SCALAR time, const size_t shotIdx) override
    {
        Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> result;
        result.resize(T::DIM);

        //		int shotIdx = timeGrid_->getShotIndex(time);
        SCALAR t_shot = timeGrid_->getShotDuration(shotIdx);     /* current duration of a whole shot*/
        SCALAR t_s_start = timeGrid_->getShotStartTime(shotIdx); /* time when this particular shot started */
        SCALAR t_s_end = timeGrid_->getShotEndTime(shotIdx);     /* time when this particular shot ends */

        assert(shotIdx < nodes_.size());

        result = nodes_[shotIdx] * (t_s_end - time) / t_shot + nodes_[shotIdx + 1] * (time - t_s_start) / t_shot;

        return result;
    }


    vector_t splineDerivative_t(const SCALAR time, const size_t shotIdx) const override
    {
        vector_t result;

        SCALAR t_shot = timeGrid_->getShotDuration(shotIdx); /* current duration of a whole shot*/

        result = (nodes_[shotIdx + 1] - nodes_[shotIdx]) / t_shot;

        return result;
    }


    vector_t splineDerivative_h_i(const SCALAR time, const size_t shotIdx) const override
    {
        vector_t result;

        SCALAR t_shot = timeGrid_->getShotDuration(shotIdx);     /* current duration of a whole shot*/
        SCALAR t_s_start = timeGrid_->getShotStartTime(shotIdx); /* time when this particular shot started */

        result = (time - t_s_start) * (nodes_[shotIdx] - nodes_[shotIdx + 1]) / (t_shot * t_shot);

        return result;
    }

    matrix_t splineDerivative_q_i(const SCALAR time, const size_t shotIdx) const override
    {
        matrix_t drv;

        SCALAR t_shot = timeGrid_->getShotDuration(shotIdx); /* current duration of a the shot*/
        SCALAR t_s_end = timeGrid_->getShotEndTime(shotIdx); /* time when this particular shot ends */

        drv.setIdentity();
        drv *= (t_s_end - time) / t_shot;

        return drv;
    }


    matrix_t splineDerivative_q_iplus1(const SCALAR time, const size_t shotIdx) const override
    {
        matrix_t drv;

        //		int shotIdx = timeGrid_->getShotIndex(time);
        SCALAR t_shot = timeGrid_->getShotDuration(shotIdx);     /* current duration of the shot*/
        SCALAR t_s_start = timeGrid_->getShotStartTime(shotIdx); /* time when this particular shot started */

        drv.setIdentity();
        drv *= (time - t_s_start) / t_shot;

        return drv;
    }


private:
    vector_array_t nodes_;  // an array of references to grid points between which is interpolated

    std::shared_ptr<tpl::TimeGrid<SCALAR>> timeGrid_;
};

}  // namespace optcon
}  // namespace ct
