/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once


namespace ct {
namespace core {

//! Derivatives using Num-Diff Codegeneration
/*!
 * Uses Num-Diff code generation to compute the Derivatives \f$ J(x_s) = \frac{df}{dx} |_{x=x_s} \f$ of
 * a regular vector-valued mathematical function \f$ y = f(x) \f$ .
 *
 * x has IN_DIM dimension and y has OUT_DIM dimension. Thus, they can be
 * scalar functions (IN_DIM = 1, OUT_DIM = 1), fixed or variable size
 * (IN_DIM = -1, OUT_DIM = -1) functions.
 *
 * \note In fact, this class is called Derivatives but computes also zero order derivatives
 *
 * @tparam IN_DIM Input dimensionality of the function (use Eigen::Dynamic (-1) for dynamic size)
 * @tparam OUT_DIM Output dimensionailty of the function (use Eigen::Dynamic (-1) for dynamic size)
 */
template <int IN_DIM, int OUT_DIM>
class DerivativesNumDiff : public Derivatives<IN_DIM, OUT_DIM, double>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Eigen::Matrix<double, IN_DIM, 1> IN_TYPE;    //!< function input vector type
    typedef Eigen::Matrix<double, OUT_DIM, 1> OUT_TYPE;  //!< function output vector type
    typedef Eigen::Matrix<double, OUT_DIM, IN_DIM> JAC_TYPE;
    typedef Eigen::Matrix<double, IN_DIM, IN_DIM> HES_TYPE;

    typedef std::function<OUT_TYPE(const IN_TYPE&)> Function;  //!< function tpye

    //! default constructor
    /*!
	 * Generates a Derivatives of a function using numerical differentiation
	 *
	 * Numerical differentiation can either use single sided or double sided derivatives.
	 * Double sided derivative can be more accurate but is almost twice as expensive.
	 *
	 * @param f function to compute Derivatives of
	 * @param doubleSidedDerivative use double sided differentiation
	 */
    DerivativesNumDiff(Function& f, bool doubleSidedDerivative = false)
        : f_(f), doubleSidedDerivative_(doubleSidedDerivative)
    {
        eps_ = sqrt(Eigen::NumTraits<double>::epsilon());
    }

    DerivativesNumDiff(const DerivativesNumDiff& arg)
        : f_(arg.f_), doubleSidedDerivative_(arg.doubleSidedDerivative_), eps_(arg.eps_)
    {
    }

    //! default destructor
    virtual ~DerivativesNumDiff() {}
    //! deep cloning
    DerivativesNumDiff* clone() const override { return new DerivativesNumDiff<IN_DIM, OUT_DIM>(*this); }
    virtual OUT_TYPE forwardZero(const Eigen::VectorXd& x) override { return f_(x); }
    //! evaluate Derivatives at a given point
    /*!
	 * @param x point at which to evaluate the Derivatives at
	 * @return Derivatives evaluated at x
	 */
    virtual JAC_TYPE jacobian(const Eigen::VectorXd& x) override
    {
        JAC_TYPE jac;
        OUT_TYPE y_ref;

        if (!doubleSidedDerivative_)
        {
            y_ref = f_(x);
        }

        for (size_t i = 0; i < x.rows(); ++i)
        {
            // inspired from http://en.wikipedia.org/wiki/Numerical_differentiation#Practical_considerations_using_floating_point_arithmetic
            double h = eps_ * std::max(std::abs(x(i)), 1.0);
            volatile double x_ph = x(i) + h;
            double dxp = x_ph - x(i);

            IN_TYPE x_perturbed = x;
            x_perturbed(i) = x_ph;

            // get evaluation of f(x,u)
            OUT_TYPE y_perturbed = f_(x_perturbed);

            if (doubleSidedDerivative_)
            {
                volatile double x_mh = x(i) - h;
                double dxm = x(i) - x_mh;

                x_perturbed = x;
                x_perturbed(i) = x_mh;
                OUT_TYPE y_perturbed_low = f_(x_perturbed);

                jac.template col(i) = (y_perturbed - y_perturbed_low) / (dxp + dxm);
            }
            else
            {
                jac.template col(i) = (y_perturbed - y_ref) / dxp;
            }
        }

        return jac;
    }

private:
    std::function<OUT_TYPE(const IN_TYPE&)> f_;  //!< the function

    bool doubleSidedDerivative_;  //!< if true, will use double sided differentiation
    double eps_;                  //!< the perturbation to apply for numerical differentiation
};
}
}
