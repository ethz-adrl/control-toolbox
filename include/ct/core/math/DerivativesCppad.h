/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "DerivativesCppadSettings.h"

namespace ct {
namespace core {

#ifdef CPPAD

//! Jacobian using Auto-Diff Codegeneration
/*!
 * Uses Auto-Diff code generation to compute the Jacobian \f$ J(x_s) = \frac{df}{dx} |_{x=x_s} \f$ of
 * a regular vector-valued mathematical function \f$ y = f(x) \f$ .
 *
 * x has IN_DIM dimension and y has OUT_DIM dimension. Thus, they can be
 * scalar functions (IN_DIM = 1, OUT_DIM = 1), fixed or variable size
 * (IN_DIM = -1, OUT_DIM = -1) functions.
 *
 * \note In fact, this class is called Jacobian but computes also zero order derivatives
 *
 * @tparam IN_DIM Input dimensionality of the function (use Eigen::Dynamic (-1) for dynamic size)
 * @tparam OUT_DIM Output dimensionailty of the function (use Eigen::Dynamic (-1) for dynamic size)
 */
template <int IN_DIM, int OUT_DIM>
class DerivativesCppad : public Derivatives<IN_DIM, OUT_DIM, double>  // double on purpose!
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef ADScalar AD_SCALAR;

    typedef Eigen::Matrix<AD_SCALAR, IN_DIM, 1> IN_TYPE_AD;    //!< function input vector type
    typedef Eigen::Matrix<AD_SCALAR, OUT_DIM, 1> OUT_TYPE_AD;  //!< function  output vector type

    typedef Eigen::Matrix<double, IN_DIM, 1> IN_TYPE_D;         //!< function input vector type double
    typedef Eigen::Matrix<double, OUT_DIM, 1> OUT_TYPE_D;       //!< function output vector type
    typedef Eigen::Matrix<double, OUT_DIM, IN_DIM> JAC_TYPE_D;  //!< Jacobian type
    typedef Eigen::Matrix<double, OUT_DIM, IN_DIM, Eigen::RowMajor>
        JAC_TYPE_ROW_MAJOR;  //!< Jocobian type in row-major format
    typedef Eigen::Matrix<double, IN_DIM, IN_DIM> HES_TYPE_D;
    typedef Eigen::Matrix<double, IN_DIM, IN_DIM, Eigen::RowMajor> HES_TYPE_ROW_MAJOR;

    typedef std::function<OUT_TYPE_AD(const IN_TYPE_AD&)> FUN_TYPE_AD;

    typedef Derivatives<IN_DIM, OUT_DIM> DerivativesBase;


    /**
     * @brief      Constructs the derivatives for autodiff without
     *             codegeneration using a FUN_TYPE_AD function
     *
     * @warning    If IN_DIM and/our OUT_DIM are set to dynamic (-1), then the
     *             actual dimensions of x and y have to be passed here.
     *
     * @param      f          The function to be autodiffed
     * @param[in]  inputDim   inputDim input dimension, must be specified if
     *                        template parameter IN_DIM is -1 (dynamic)
     * @param[in]  outputDim  outputDim output dimension, must be specified if
     *                        template parameter IN_DIM is -1 (dynamic)
     */
    DerivativesCppad(FUN_TYPE_AD& f, int inputDim = IN_DIM, int outputDim = OUT_DIM)
        : DerivativesBase(), adStdFun_(f), inputDim_(inputDim), outputDim_(outputDim)
    {
        update(f, inputDim, outputDim);
    }

    //! copy constructor
    DerivativesCppad(const DerivativesCppad& arg)
        : DerivativesBase(arg), adStdFun_(arg.adStdFun_), inputDim_(arg.inputDim_), outputDim_(arg.outputDim_)
    {
        adCppadFun_ = arg.adCppadFun_;
    }


    //! update the Jacobian with a new function
    /*!
     * \warning If IN_DIM and/our OUT_DIM are set to dynamic (-1), then the actual dimensions of
     * x and y have to be passed here.
     *
     * @param f new function to compute Jacobian of
     * @param inputDim input dimension, must be specified if template parameter IN_DIM is -1 (dynamic)
     * @param outputDim output dimension, must be specified if template parameter IN_DIM is -1 (dynamic)
     */
    void update(FUN_TYPE_AD& f, const size_t inputDim = IN_DIM, const size_t outputDim = OUT_DIM)
    {
        adStdFun_ = f;
        outputDim_ = outputDim;
        inputDim_ = inputDim;
        if (outputDim_ > 0 && inputDim_ > 0)
            recordAd();
    }

    //! destructor
    virtual ~DerivativesCppad() {}
    //! deep cloning of Jacobian
    DerivativesCppad* clone() const { return new DerivativesCppad<IN_DIM, OUT_DIM>(*this); }
    virtual OUT_TYPE_D forwardZero(const Eigen::VectorXd& x) { return adCppadFun_.Forward(0, x); }
    virtual JAC_TYPE_D jacobian(const Eigen::VectorXd& x)
    {
        if (outputDim_ <= 0)
            throw std::runtime_error("Outdim dim smaller 0; Define output dim in DerivativesCppad constructor");


        Eigen::VectorXd jac = adCppadFun_.Jacobian(x);

        JAC_TYPE_D out(outputDim_, x.rows());
        out = JAC_TYPE_ROW_MAJOR::Map(jac.data(), outputDim_, x.rows());
        return out;
    }

    virtual void sparseJacobian(const Eigen::VectorXd& x,
        Eigen::VectorXd& jac,
        Eigen::VectorXi& iRow,
        Eigen::VectorXi& jCol)
    {
        if (outputDim_ <= 0)
            throw std::runtime_error("Outdim dim smaller 0; Define output dim in DerivativesCppad constructor");

        jac = adCppadFun_.SparseJacobian(x);
    }

    virtual Eigen::VectorXd sparseJacobianValues(const Eigen::VectorXd& x)
    {
        if (outputDim_ <= 0)
            throw std::runtime_error("Outdim dim smaller 0; Define output dim in DerivativesCppad constructor");

        return adCppadFun_.SparseJacobian(x);
    }


    virtual HES_TYPE_D hessian(const Eigen::VectorXd& x, const Eigen::VectorXd& lambda)
    {
        if (outputDim_ <= 0)
            throw std::runtime_error("Outdim dim smaller 0; Define output dim in DerivativesCppad constructor");

        Eigen::VectorXd hessian = adCppadFun_.Hessian(x, lambda);
        HES_TYPE_D out(x.rows(), x.rows());
        out = HES_TYPE_ROW_MAJOR::Map(hessian.data(), x.rows(), x.rows());
        return out;
    }

    virtual void sparseHessian(const Eigen::VectorXd& x,
        const Eigen::VectorXd& lambda,
        Eigen::VectorXd& hes,
        Eigen::VectorXi& iRow,
        Eigen::VectorXi& jCol)
    {
        if (outputDim_ <= 0)
            throw std::runtime_error("Outdim dim smaller 0; Define output dim in DerivativesCppad constructor");

        hes = adCppadFun_.SparseHessian(x, lambda);
    }


    virtual Eigen::VectorXd sparseHessianValues(const Eigen::VectorXd& x, const Eigen::VectorXd& lambda)
    {
        if (outputDim_ <= 0)
            throw std::runtime_error("Outdim dim smaller 0; Define output dim in DerivativesCppad constructor");

        return adCppadFun_.SparseHessian(x, lambda);
    }

private:
    /**
     * @brief      Records the auto-diff terms
     */
    void recordAd()
    {
        // input vector, needs to be dynamic size
        Eigen::Matrix<AD_SCALAR, Eigen::Dynamic, 1> x(inputDim_);

        // declare x as independent
        CppAD::Independent(x);

        // output vector, needs to be dynamic size
        Eigen::Matrix<AD_SCALAR, Eigen::Dynamic, 1> y(outputDim_);

        y = adStdFun_(x);

        // store operation sequence in f: x -> y and stop recording
        CppAD::ADFun<double> fAd(x, y);

        fAd.optimize();

        std::cout << "AD FUn recorded" << std::endl;

        adCppadFun_ = fAd;
    }

    std::function<OUT_TYPE_AD(const IN_TYPE_AD&)> adStdFun_;

    int inputDim_;   //! function input dimension
    int outputDim_;  //! function output dimension

    CppAD::ADFun<double> adCppadFun_;
};

#endif

} /* namespace core */
} /* namespace ct */
