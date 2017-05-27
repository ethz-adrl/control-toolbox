#ifndef DMS_STAB_DIMENSIONS_HPP_
#define DMS_STAB_DIMENSIONS_HPP_

#include <vector>
#include <Eigen/Dense>

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM>
class StabDimensions {

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef ct::core::StateVector<STATE_DIM> state_vector_t;
	typedef ct::core::StateVectorArray<STATE_DIM> state_vector_array_t;

	typedef Eigen::Matrix<double, STATE_DIM, STATE_DIM> state_matrix_t;
	typedef ct::core::StateMatrixArray<STATE_DIM> state_matrix_array_t;

	typedef Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> state_control_matrix_t;
	typedef ct::core::StateControlMatrixArray<STATE_DIM, CONTROL_DIM> state_control_matrix_array_t;

	typedef ct::core::ControlVector<CONTROL_DIM> control_vector_t;
	typedef ct::core::ControlVectorArray<CONTROL_DIM> control_vector_array_t;

	typedef Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> control_matrix_t;
	typedef ct::core::ControlMatrixTrajectory<CONTROL_DIM> control_matrix_array_t;

	typedef Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> control_feedback_t;
	typedef ct::core::FeedbackArray<STATE_DIM, CONTROL_DIM> control_feedback_array_t;

    typedef Eigen::Matrix<double, STATE_DIM*STATE_DIM , 1 > state_matrix_vectorized_t;

	typedef double time_t;
	typedef ct::core::TimeArray time_array_t;
};

} // namespace optcon
} // namespace ct

#endif
