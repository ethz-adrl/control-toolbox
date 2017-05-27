/*
 * LinearSpliner.hpp
 *
 * Created: 14.01.2016
 * Author: mgiftthaler
 *
 * */

/*
 * How to improve linear splining:
 * Problem definition: there is currently a mismatch between assumptions for cost gradient calculation and the way the forward integration is done.
 * Cost gradient assums piecewise constant controls and state. But forward propagation does not? Can that be a problem?
 * Should we include sub-gradients into the cost gradient calculation?
 *
 * TODO (as of 26.10.2016 -- is this still an issue?)
 * */

#ifndef LINEAR_SPLINER_DMS_HPP_
#define LINEAR_SPLINER_DMS_HPP_

#include "ct/optcon/dms/dms_core/spline/SplinerBase.hpp"
#include <ct/optcon/dms/dms_core/TimeGrid.hpp>

namespace ct {
namespace optcon {

template<class T>
class LinearSpliner : public SplinerBase<T>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef T vector_t;
	typedef std::vector<vector_t, Eigen::aligned_allocator<vector_t>> vector_array_t;
	typedef Eigen::Matrix<double, T::DIM, T::DIM> matrix_t;

	LinearSpliner() = delete;

	LinearSpliner(std::shared_ptr<TimeGrid> grid):
		timeGrid_(grid)
	{}

	virtual ~LinearSpliner(){}

	void computeSpline(const vector_array_t& points) override {
		nodes_ = points;
	}

	// evaluate spline and return vector at interpolation time
	virtual vector_t evalSpline (const double time,  const size_t shotIdx) override {

		Eigen::VectorXd result;
		result.resize(T::DIM);

		//		int shotIdx = timeGrid_->getShotIndex(time);
		double t_shot = timeGrid_->getShotDuration(shotIdx);		/* current duration of a whole shot*/
		double t_s_start = timeGrid_->getShotStartTime(shotIdx);	/* time when this particular shot started */
		double t_s_end = timeGrid_->getShotEndTime(shotIdx);		/* time when this particular shot ends */

		assert(shotIdx < nodes_.size());

		result = nodes_[shotIdx]*(t_s_end - time) / t_shot
				+ nodes_[shotIdx+1]*(time-t_s_start) / t_shot;

		return result;
	}


	virtual vector_t splineDerivative_t (const double time,  const size_t shotIdx) const override {

		vector_t result;

		double t_shot = timeGrid_->getShotDuration(shotIdx);		/* current duration of a whole shot*/

		result = (nodes_[shotIdx+1]- nodes_[shotIdx]) / t_shot;

		return result;
	}


	virtual vector_t splineDerivative_h_i (const double time, const size_t shotIdx) const override {

		vector_t result;

		double t_shot = timeGrid_->getShotDuration(shotIdx);		/* current duration of a whole shot*/
		double t_s_start = timeGrid_->getShotStartTime(shotIdx);	/* time when this particular shot started */

		result = (time-t_s_start)*(nodes_[shotIdx] - nodes_[shotIdx + 1])/(t_shot*t_shot);

		return result;
	}

	virtual matrix_t splineDerivative_q_i (const double time,  const size_t shotIdx) const override {

		matrix_t drv;

		double t_shot = timeGrid_->getShotDuration(shotIdx);		/* current duration of a the shot*/
		double t_s_end = timeGrid_->getShotEndTime(shotIdx);		/* time when this particular shot ends */

		drv.setIdentity();
		drv *= (t_s_end - time) / t_shot;

		return drv;
	}


	virtual matrix_t splineDerivative_q_iplus1(const double time,  const size_t shotIdx) const override {

		matrix_t drv;

		//		int shotIdx = timeGrid_->getShotIndex(time);
		double t_shot = timeGrid_->getShotDuration(shotIdx);		/* current duration of the shot*/
		double t_s_start = timeGrid_->getShotStartTime(shotIdx);	/* time when this particular shot started */

		drv.setIdentity();
		drv *= (time-t_s_start) / t_shot;

		return drv;
	}


private:

	vector_array_t nodes_;	// an array of references to grid points between which is interpolated

	std::shared_ptr<TimeGrid> timeGrid_;

};

} // namespace optcon
} // namespace ct

#endif
