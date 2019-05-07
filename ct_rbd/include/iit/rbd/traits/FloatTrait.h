/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once
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
	inline static Scalar sqrt(const Scalar& x) {return std::sqrt(x); }

	template <int Rows, int Cols>
	inline static Eigen::Matrix<Scalar, Cols, 1> solve(const Eigen::Matrix<Scalar, Rows, Cols>& A, const Eigen::Matrix<Scalar, Rows, 1>& b)
	{
		return A.inverse()*b;
	}

};

}
}
