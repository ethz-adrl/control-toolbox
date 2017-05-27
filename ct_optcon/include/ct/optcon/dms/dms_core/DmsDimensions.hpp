#ifndef DMS_DIMENSIONS_HPP_
#define DMS_DIMENSIONS_HPP_

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM>
class DmsDimensions {

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef ct::core::StateVector<STATE_DIM> state_vector_t;
	typedef ct::core::StateVectorArray<STATE_DIM> state_vector_array_t;

	typedef Eigen::Matrix<double, STATE_DIM, STATE_DIM> state_matrix_t;
	typedef ct::core::StateMatrixArray<STATE_DIM> state_matrix_array_t;

	typedef Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> state_control_matrix_t;
	typedef ct::core::StateControlMatrixArray<STATE_DIM, CONTROL_DIM> state_control_matrix_array_t;

	typedef Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> control_state_matrix_t;

	typedef ct::core::ControlVector<CONTROL_DIM> control_vector_t;
	typedef ct::core::ControlVectorArray<CONTROL_DIM> control_vector_array_t;

	typedef Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> control_matrix_t;
	typedef ct::core::ControlMatrixTrajectory<CONTROL_DIM> control_matrix_array_t;

	typedef ct::core::Time time_t;
	typedef ct::core::TimeArray time_array_t;

private:

};

} // namespace optcon
} // namespace ct

#endif /* DMS_DIMENSIONS_HPP_ */
