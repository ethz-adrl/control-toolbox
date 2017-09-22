/***********************************************************************************
Copyright (c) 2017, Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo,
Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be used
      to endorse or promote products derived from this software without specific
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/

#ifndef INCLUDE_CT_CORE_FUNCTION_CPPAD_UTILS_H_
#define INCLUDE_CT_CORE_FUNCTION_CPPAD_UTILS_H_

#include "DerivativesCppadSettings.h"

namespace ct {
namespace core {

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
class CppadUtils
{
public:
    typedef ADScalar AD_SCALAR;
    typedef ADCGScalar CG_SCALAR; //!< CG_SCALAR  type
    typedef ADCGValueType CG_VALUE_TYPE; //!< autodiff scalar type
                                       
    typedef Eigen::Matrix<AD_SCALAR, IN_DIM, 1> IN_TYPE_AD; //!< function input vector type
    typedef Eigen::Matrix<AD_SCALAR, OUT_DIM, 1> OUT_TYPE_AD; //!< function  output vector type 

    typedef Eigen::Matrix<CG_SCALAR, IN_DIM, 1> IN_TYPE_CG; //!< function input vector type
    typedef Eigen::Matrix<CG_SCALAR, OUT_DIM, 1> OUT_TYPE_CG; //!< function  output vector type                 

    typedef std::function<OUT_TYPE_CG(const IN_TYPE_CG&)> FUN_TYPE_CG; //!< function type
    typedef std::function<OUT_TYPE_AD(const IN_TYPE_AD&)> FUN_TYPE_AD;

    /**
     * @brief      Contructs the derivatives for codegeneration using a
     *             FUN_TYPE_CG function
     * @warning    If IN_DIM and/our OUT_DIM are set to dynamic (-1), then the
     *             actual dimensions of x and y have to be passed here.
     *
     * @param      f          The function to be autodiffed
     * @param[in]  inputDim   inputDim input dimension, must be specified if
     *                        template parameter IN_DIM is -1 (dynamic)
     * @param[in]  outputDim  outputDim output dimension, must be specified if
     *                        template parameter IN_DIM is -1 (dynamic)
     */
    CppadUtils(
        FUN_TYPE_CG& f, 
        int inputDim = IN_DIM, 
        int outputDim = OUT_DIM) 
    :
        fCgStd_(f),
        inputDim_(inputDim),
        outputDim_(outputDim)
    {
        update(f, inputDim, outputDim);
        // if(outputDim_ > 0 && inputDim_ > 0)
        //     recordCg();
    }

    /*
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
     
    CppadUtils(
        FUN_TYPE_AD& f, 
        int inputDim = IN_DIM, 
        int outputDim = OUT_DIM) 
    :
        fAdStd_(f),
        inputDim_(inputDim),
        outputDim_(outputDim)
    {
        if(outputDim_ > 0 && inputDim_ > 0)
            recordAd();
    }

    //! copy constructor
    CppadUtils(const CppadUtils& arg) 
    :
        fCgStd_(arg.fCgStd_),
        fAdStd_(arg.fAdStd_),
        inputDim_(arg.inputDim_),
        outputDim_(arg.outputDim_)
    {
        fCgCppad_ = arg.fCgCppad_;
        fAdCppad_ = arg.fAdCppad_;
    }

    //! destructor
    virtual ~CppadUtils()
    {
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
    void update(FUN_TYPE_CG& f, const size_t inputDim = IN_DIM, const size_t outputDim = OUT_DIM)
    {
        std::cout << "Updating CG fun" << std::endl;
        fCgStd_ = f;
        outputDim_ = outputDim;
        inputDim_ = inputDim;
        if(outputDim_ > 0 && inputDim_ > 0)
        {
            recordCg();
            updateDerived();
        }
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
        fAdStd_ = f;
        outputDim_ = outputDim;
        inputDim_ = inputDim;
        if(outputDim_ > 0 && inputDim_ > 0)
        {
            recordAd();
            updateDerived();
        }
    }

    virtual void updateDerived(){};

    //! deep cloning of Jacobian
    CppadUtils* clone() const{
        return new CppadUtils<IN_DIM, OUT_DIM>(*this);
    }

protected:
    //! record the Auto-Diff terms for code generation
    void recordCg()
    {
        // input vector, needs to be dynamic size
        Eigen::Matrix<CG_SCALAR, Eigen::Dynamic, 1> x(inputDim_);

        // declare x as independent
        CppAD::Independent(x);

        // output vector, needs to be dynamic size
        Eigen::Matrix<CG_SCALAR, Eigen::Dynamic, 1> y(outputDim_);

        y = fCgStd_(x);

        // store operation sequence in f: x -> y and stop recording
        CppAD::ADFun<CG_VALUE_TYPE> fCodeGen(x, y);

        fCodeGen.optimize();

        fCgCppad_ = fCodeGen;
    }

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

        y = fAdStd_(x);

        // store operation sequence in f: x -> y and stop recording
        CppAD::ADFun<double> fAd(x, y);

        fAd.optimize();

        fAdCppad_ = fAd;        
    }

    std::function<OUT_TYPE_CG(const IN_TYPE_CG&)> fCgStd_; //! the function
    std::function<OUT_TYPE_AD(const IN_TYPE_AD&)> fAdStd_;

    int inputDim_; //! function input dimension
    int outputDim_; //! function output dimension

    CppAD::ADFun<CG_VALUE_TYPE> fCgCppad_; //!  auto-diff function
    CppAD::ADFun<double> fAdCppad_;
};

} /* namespace core */
} /* namespace ct */

#endif /* INCLUDE_CT_CORE_FUNCTION_CPPAD_UTILS_H_ */
