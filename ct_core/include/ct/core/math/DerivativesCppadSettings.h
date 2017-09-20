/***********************************************************************************
Copyright (c) 2016, Agile & Dexterous Robotics Lab, ETH ZURICH. All rights reserved.

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

#ifndef  CT_CORE_MATH_DERIVATIVESCPPAD_SETTINGS_H_
#define  CT_CORE_MATH_DERIVATIVESCPPAD_SETTINGS_H_

#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>

namespace ct {
namespace core {

/**
 * @ingroup    NLP
 *
 * @brief      Contains the NLP solver settings
 */
class DerivativesCppadSettings
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef enum CompilerType {GCC = 0, CLANG = 1, num_types_compiler} Compiler_t;

    /**
     * @brief      Default constructor, set default settings
     */
    DerivativesCppadSettings() :
    multiThreading_(false),
    createForwardZero_(false),
    createForwardOne_(false),
    createReverseOne_(false),
    createReverseTwo_(false),
    createJacobian_(false),
    createSparseJacobian_(false),
    createHessian_(false),
    createSparseHessian_(false),
    maxAssignements_(20000),
    compiler_(GCC)
    {}

    bool multiThreading_;
    bool createForwardZero_;
    bool createForwardOne_;
    bool createReverseOne_;
    bool createReverseTwo_;
    bool createJacobian_;
    bool createSparseJacobian_;
    bool createHessian_;
    bool createSparseHessian_;
    size_t maxAssignements_;
    CompilerType compiler_;
    /**
     * @brief      Prints out settings
     */
    void print()
    {
        std::cout<<"============================================================="<<std::endl;
        std::cout<<"\tCPPAD Derivatives Settings: "<<std::endl;
        std::cout<<"============================================================="<<std::endl;

        if(multiThreading_)
            std::cout << "Enabling multithreading in JIT lib" << std::endl;
        if(createForwardZero_)
            std::cout << "Generating Forward Zero in JIT lib" << std::endl;
        if(createReverseOne_)
            std::cout << "Generating Reverse One in JIT lib" << std::endl;
        if(createReverseTwo_)
            std::cout << "Generating Reverse Two in JIT lib" << std::endl;
        if(createJacobian_)
            std::cout << "Generating Jacobian in JIT lib" << std::endl;
        if(createSparseJacobian_)
            std::cout << "Generating Sparse Jacobian in JIT lib" << std::endl;
        if(createHessian_)
            std::cout << "Generating Hessian in JIT lib" << std::endl;
        if(createSparseHessian_)
            std::cout << "Generating Sparse Hessian in JIT lib" << std::endl;

        std::cout << "Using " + compilerToString[compiler_] + " to compile the generated derivatives" << std::endl;

        std::cout << "Max Assignements per generated Function:" << maxAssignements_ << std::endl;
    }

     /**
     * @brief      Checks whether to settings are filled with meaningful values
     *
     * @return     Returns true of the parameters are ok
     */
    bool parametersOk() const
    {
        return true;
    }

    /**
     * @brief      Loads the settings from a .info file
     *
     * @param[in]  filename  The filename
     * @param[in]  verbose   True if parameters to be printed out
     * @param[in]  ns        The namespace in the .info fil
     */
    void load(const std::string& filename, bool verbose = true, const std::string& ns = "solver")
    {
        boost::property_tree::ptree pt;
        boost::property_tree::read_info(filename, pt);

        multiThreading_         = pt.get<bool>(ns + ".MultiThreading");
        createForwardZero_      = pt.get<bool>(ns + ".CreateForwardZero");
        createForwardOne_       = pt.get<bool>(ns + ".CreateForwardOne");
        createReverseOne_       = pt.get<bool>(ns + ".CreateReverseOne");
        createReverseTwo_       = pt.get<bool>(ns + ".CreateReverseTwo");
        createJacobian_         = pt.get<bool>(ns + ".CreateJacobian");
        createSparseJacobian_   = pt.get<bool>(ns + ".CreateSparseJacobian");
        createHessian_          = pt.get<bool>(ns + ".CreateHessian");
        createSparseHessian_    = pt.get<bool>(ns + ".CreateSparseHessian");
        maxAssignements_        = pt.get<unsigned int>(ns + ".MaxAssignements");

        std::string compilerStr = pt.get<std::string>(ns + ".Compiler");

        if (stringTocompiler.find(compilerStr) != stringTocompiler.end())
            compiler_ = stringTocompiler[compilerStr];
        else
        {
            std::cout << "Invalid compiler specified in config, should be one of the following:" << std::endl;

            for(auto it = stringTocompiler.begin(); it != stringTocompiler.end(); it++)
                std::cout << it->first << std::endl;
        }                

        if (verbose)
        {
            std::cout << "Loaded Derivatives CPPAD settings from "<<filename<<": "<<std::endl;
            print();
        }
    }

private:
    std::map<CompilerType, std::string> compilerToString = {{GCC, "GCC"}, {CLANG, "CLANG"}};
    std::map<std::string, CompilerType> stringTocompiler = {{"GCC", GCC}, {"CLANG", CLANG}};
};


} // namespace core
} // namespace ct

#endif // CT_CORE_MATH_DERIVATIVESCPPAD_SETTINGS_H_
