/*
 * ZeroOrderHoldSpliner.hpp
 *
 * Created: 14.01.2016
 * Author: mgiftthaler
 *
 * */

#ifndef DMS_ZERO_ORDER_HOLD_SPLINER_HPP_
#define DMS_ZERO_ORDER_HOLD_SPLINER_HPP_

#include "ct/optcon/dms/dms_core/spline/SplinerBase.hpp"
#include <ct/optcon/dms/dms_core/TimeGrid.hpp>

namespace ct {
namespace optcon {

template<class T>
class ZeroOrderHoldSpliner: public SplinerBase<T>
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef T vector_t;
	typedef Eigen::Matrix<double, T::DIM, T::DIM> matrix_t;
	typedef std::vector<vector_t, Eigen::aligned_allocator<vector_t>> vector_array_t;

	ZeroOrderHoldSpliner() = delete;

	ZeroOrderHoldSpliner(std::shared_ptr<TimeGrid> grid):
		timeGrid_(grid)
	{}

	virtual ~ZeroOrderHoldSpliner(){}

	void computeSpline(const vector_array_t& points) override {

		for(size_t i = 0; i<points.size(); i++)
		{
			assert(points[i] == points[i]);
		}

		zOholds_ = points;

		for(size_t i = 0; i<zOholds_.size(); i++)
		{
			assert(zOholds_[i] == zOholds_[i]);
		}
	}


	// evaluate spline and return vector at interpolation time
	virtual vector_t evalSpline (const double time, const size_t shotIdx) override {
		assert(shotIdx < zOholds_.size() );
		assert(zOholds_[shotIdx] == zOholds_[shotIdx]);
		return zOholds_[shotIdx];
	}

	virtual vector_t splineDerivative_t (const double time,  const size_t shotIdx) const override {
		return vector_t::Zero();
	}

	virtual vector_t splineDerivative_h_i(const double time, const size_t shotIdx) const override {
		return vector_t::Zero();
	}

	virtual matrix_t splineDerivative_q_i (const double time,  const size_t shotIdx) const override {
		return matrix_t::Identity();
	}

	virtual matrix_t splineDerivative_q_iplus1(const double time,  const size_t shotIdx) const override {
		return matrix_t::Zero();
	}


private:
	// zero order hold variables
	vector_array_t zOholds_;

	std::shared_ptr<TimeGrid> timeGrid_;

};

} // namespace optcon
} // namespace ct

#endif
