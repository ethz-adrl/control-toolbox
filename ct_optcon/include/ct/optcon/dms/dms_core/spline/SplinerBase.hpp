/*
 * SplinerBase.hpp
 *
 * Created: 14.01.2016
 * Author: mgiftthaler
 *
 * */

#ifndef SPLINER_BASE_DMS_HPP_
#define SPLINER_BASE_DMS_HPP_

namespace ct {
namespace optcon {

template<class T>
class SplinerBase{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	SplinerBase(){};

	virtual ~SplinerBase(){};

	typedef T vector_t;
	typedef Eigen::Matrix<double, T::DIM, T::DIM> matrix_t;
	typedef std::vector<vector_t, Eigen::aligned_allocator<vector_t>> vector_array_t;


	virtual void computeSpline(const vector_array_t& points) = 0;

	virtual vector_t evalSpline (const double time, const size_t shotIdx) = 0;

	virtual vector_t splineDerivative_t (const double time,  const size_t shotIdx) const = 0;

	virtual vector_t splineDerivative_h_i(const double time, const size_t shotIdx) const = 0;

	virtual matrix_t splineDerivative_q_i (const double time,  const size_t shotIdx) const = 0;

	virtual matrix_t splineDerivative_q_iplus1(const double time,  const size_t shotIdx) const = 0;

};

} // namespace optcon
} // namespace ct

#endif
