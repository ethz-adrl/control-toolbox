/*
 * DoubleTrait.hpp
 *
 *  Created on: Nov 7, 2016
 *      Author: neunertm
 */

#ifndef INCLUDE_EXTERNAL_HYQ_TRAITS_FLOATTRAIT_HPP_
#define INCLUDE_EXTERNAL_HYQ_TRAITS_FLOATTRAIT_HPP_

namespace iit {
namespace rbd {



struct FloatTrait {

	typedef float Scalar;

	inline static Scalar sin(const Scalar& x) { return std::sin(x); }
	inline static Scalar cos(const Scalar& x) { return std::cos(x); }
	inline static Scalar tan(const Scalar& x) { return std::tan(x); }
	inline static Scalar sinh(const Scalar& x) { return std::sinh(x); }
	inline static Scalar cosh(const Scalar& x) { return std::cosh(x); }
	inline static Scalar tanh(const Scalar& x) { return std::tanh(x); }
	inline static Scalar exp(const Scalar& x) { return std::exp(x); }
	inline static Scalar fabs(const Scalar& x) { return std::fabs(x); }

	template <int Rows, int Cols>
	inline static Eigen::Matrix<Scalar, Cols, 1> solve(const Eigen::Matrix<Scalar, Rows, Cols>& A, const Eigen::Matrix<Scalar, Rows, 1>& b)
	{
		return A.inverse()*b;
	}

};

}
}


#endif /* INCLUDE_EXTERNAL_HYQ_TRAITS_FLOATTRAIT_HPP_ */