/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>

namespace ct {
namespace core {

//! General interface class for a Derivatives
/*!
 * Interface for a general Derivatives of a vector-valued function. Can be used either
 * with fixed size or dynamic size data types
 *
 * @tparam IN_DIM input dimension of function (use Eigen::Dynamic (-1) for dynamic size)
 * @tparam OUT_DIM output dimension of function (use Eigen::Dynamic (-1) for dynamic size)
 * @tparam SCALAR scalar data type
 */
template <int IN_DIM, int OUT_DIM, typename SCALAR = double>
class Derivatives
{
public:
    typedef Eigen::Matrix<double, IN_DIM, 1> IN_TYPE;    //!< function input vector type
    typedef Eigen::Matrix<double, OUT_DIM, 1> OUT_TYPE;  //!< function output vector type
    typedef Eigen::Matrix<double, OUT_DIM, IN_DIM> JAC_TYPE;
    typedef Eigen::Matrix<double, IN_DIM, IN_DIM> HES_TYPE;

    Derivatives(){};

    //! default destructor
    virtual ~Derivatives(){};

    //! deep copy for derived classes
    virtual Derivatives<IN_DIM, OUT_DIM, SCALAR>* clone() const = 0;

    /**
     * @brief      Evaluates the method itself
     *
     * @param[in]  x     The point of evaluation
     *
     * @return     The evalution of the method
     */
    virtual OUT_TYPE forwardZero(const Eigen::VectorXd& x)
    {
        throw std::runtime_error("FUNCTION EVALUATION NOT IMPLEMENTED FOR THIS TYPE OF DERIVATIVE");
    }

    /**
     * @brief      Evaluates the jacobian with respect to the input
     *
     * @param[in]  x     The point of evaluation
     *
     * @return     The evaluated jacobian
     */
    virtual JAC_TYPE jacobian(const Eigen::VectorXd& x)
    {
        throw std::runtime_error("JACOBIAN EVALUATION NOT IMPLEMENTED FOR THIS TYPE OF DERIVATIVE");
    }

    /**
     * @brief      Returns the evaluated jacobian in sparse format
     *
     * @param[in]  x     The point of evaluation
     * @param[out]      jac   The non zero values of the jacobian
     * @param[out]      iRow  The row indices of the non zero values
     * @param[out]      jCol  The column indices of the non zero values
     */
    virtual void sparseJacobian(const Eigen::VectorXd& x,
        Eigen::VectorXd& jac,
        Eigen::VectorXi& iRow,
        Eigen::VectorXi& jCol)
    {
        throw std::runtime_error("SPARSE JACOBIAN EVALUATION NOT IMPLEMENTED FOR THIS TYPE OF DERIVATIVE");
    }

    /**
     * @brief      Returns the non zero values of the jacobian
     *
     * @param[in]  x     The point of evaluation
     *
     * @return     The non zeros values of the jacobian
     */
    virtual Eigen::VectorXd sparseJacobianValues(const Eigen::VectorXd& x)
    {
        throw std::runtime_error("SPARSE JACOBIAN EVALUATION NOT IMPLEMENTED FOR THIS TYPE OF DERIVATIVE");
    }

    /**
     * @brief      Evaluates the hessian (2nd order derivatives with respect to
     *             input) of the method. In case of a vector valued function,
     *             the method returns the weighted sum of the hessians with
     *             weights w
     *
     * @param[in]  x       The point of evaluation
     * @param[in]  lambda  The weights of the sum
     *
     * @return     The evaluated hessian
     */
    virtual HES_TYPE hessian(const Eigen::VectorXd& x, const Eigen::VectorXd& lambda)
    {
        throw std::runtime_error("HESSIAN EVALUATION NOT IMPLEMENTED FOR THIS TYPE OF DERIVATIVE");
    }

    /**
     * @brief      Returns the weighted sum of hessian of the problem in sparse
     *             format
     *
     * @param[in]  x       The point of evaluation
     * @param[in]  lambda  The weights of the sum of the hessian
     * @param      hes     The non zero values of the hessian
     * @param      iRow    The row indices of the non zero values
     * @param      jCol    The column indices of the non zero values
     */
    virtual void sparseHessian(const Eigen::VectorXd& x,
        const Eigen::VectorXd& lambda,
        Eigen::VectorXd& hes,
        Eigen::VectorXi& iRow,
        Eigen::VectorXi& jCol)
    {
        throw std::runtime_error("SPARSE HESSIAN NOT IMPLEMENTED FOR THIS TYPE OF DERIVATIVE");
    }

    /**
     * @brief      Returns the non zero elements of the hessian of the problem
     *
     * @param[in]  x       The point of evaluation
     * @param[in]  lambda  The weights of the sum of the hessian
     *
     * @return     { description_of_the_return_value }
     */
    virtual Eigen::VectorXd sparseHessianValues(const Eigen::VectorXd& x, const Eigen::VectorXd& lambda)
    {
        throw std::runtime_error("SPARSE HESSIAN EVALUATION NOT IMPLEMENTED FOR THIS TYPE OF DERIVATIVE");
    }
};

} /* namespace core */
} /* namespace ct */
