/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

/**
 * @ingroup    DMS
 *
 * @brief      Abstract base class for the control input splining between the
 *             DMS shots
 *
 * @tparam     T     The vector type which will be splined
 */
template <class T, typename SCALAR = double>
class SplinerBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
	 * @brief      Default constructor
	 */
    SplinerBase() = default;

    /**
	 * @brief      Destructor
	 */
    virtual ~SplinerBase() = default;

    typedef T vector_t;
    typedef Eigen::Matrix<SCALAR, T::DIM, T::DIM> matrix_t;
    typedef std::vector<vector_t, Eigen::aligned_allocator<vector_t>> vector_array_t;


    /**
	 * @brief      Updates the vector on the shots
	 *
	 * @param[in]  points  Updated vector array
	 */
    virtual void computeSpline(const vector_array_t& points) = 0;

    /**
	 * @brief      Depending on the spline type, this method evaluates the
	 *             control input between the shots
	 *
	 * @param[in]  time     The evaluation time
	 * @param[in]  shotIdx  The shot number
	 *
	 * @return     The splined vector
	 */
    virtual vector_t evalSpline(const SCALAR time, const size_t shotIdx) = 0;

    /**
	 * @brief      Returns the spline derivatives with respect to time
	 *
	 * @param[in]  time     The evaluation time
	 * @param[in]  shotIdx  The shot number
	 *
	 * @return     The time derivative
	 */
    virtual vector_t splineDerivative_t(const SCALAR time, const size_t shotIdx) const = 0;

    /**
	 * @brief      Returns the spline derivatives with respect to the time
	 *             segment between the shots
	 *
	 * @param[in]  time     The evaluation time
	 * @param[in]  shotIdx  The shot number
	 *
	 * @return     The resulting derivative
	 */
    virtual vector_t splineDerivative_h_i(const SCALAR time, const size_t shotIdx) const = 0;

    /**
	 * @brief      Return the spline derivative with respect to the control
	 *             input at shot i
	 *
	 * @param[in]  time     The evaluation time
	 * @param[in]  shotIdx  The shot number
	 *
	 * @return     The resulting derivative
	 */
    virtual matrix_t splineDerivative_q_i(const SCALAR time, const size_t shotIdx) const = 0;

    /**
	 * @brief      Returns the spline derivative with respect to the control
	 *             input at shot i+1
	 *
	 * @param[in]  time     The evaluation time
	 * @param[in]  shotIdx  The shot number
	 *
	 * @return     The resulting derivative
	 */
    virtual matrix_t splineDerivative_q_iplus1(const SCALAR time, const size_t shotIdx) const = 0;
};

}  // namespace optcon
}  // namespace ct
