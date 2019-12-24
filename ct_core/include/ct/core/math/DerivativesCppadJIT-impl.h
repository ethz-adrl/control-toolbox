/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#ifdef CPPADCG

namespace ct {
namespace core {


template <int IN_DIM, int OUT_DIM>
DerivativesCppadJIT<IN_DIM, OUT_DIM>::DerivativesCppadJIT(FUN_TYPE_CG& f, int inputDim, int outputDim)
    : DerivativesBase(),
      cgStdFun_(f),
      inputDim_(inputDim),
      outputDim_(outputDim),
      compiled_(false),
      libName_(""),
      dynamicLib_(nullptr)
#ifdef LLVM_VERSION_MAJOR
      ,
      llvmModelLib_(nullptr)
#endif
{
    update(f, inputDim, outputDim);
}

template <int IN_DIM, int OUT_DIM>
DerivativesCppadJIT<IN_DIM, OUT_DIM>::DerivativesCppadJIT(const DerivativesCppadJIT& arg)
    : DerivativesBase(arg),
      cgStdFun_(arg.cgStdFun_),
      inputDim_(arg.inputDim_),
      outputDim_(arg.outputDim_),
      compiled_(arg.compiled_),
      libName_(arg.libName_),
      dynamicLib_(nullptr)
#ifdef LLVM_VERSION_MAJOR
      ,
      llvmModelLib_(nullptr)
#endif
{
    cgCppadFun_ = arg.cgCppadFun_;
    if (compiled_)
    {
        if (arg.dynamicLib_)  // in case of dynamic libraries
        {
            dynamicLib_ = internal::CGHelpers::loadDynamicLibCppad<double>(libName_);
            model_ = std::shared_ptr<CppAD::cg::GenericModel<double>>(dynamicLib_->model(libName_));
        }
#ifdef LLVM_VERSION_MAJOR
        else if (arg.llvmModelLib_)  // in case of regular JIT without dynamic lib
        {
            throw std::runtime_error("DerivativesCppadJIT: cloning of LLVM-JIT libraries is currently not supported.");
            llvmModelLib_ = arg.llvmModelLib_;  // TODO: this is not clean, we need to properly clone the llvm model lib
            model_ = std::shared_ptr<CppAD::cg::GenericModel<double>>(llvmModelLib_->model(libName_));
        }
#endif
        else
            throw std::runtime_error("DerivativesCppadJIT: undefined behaviour in copy constructor.");
    }
}

template <int IN_DIM, int OUT_DIM>
void DerivativesCppadJIT<IN_DIM, OUT_DIM>::update(FUN_TYPE_CG& f, const size_t inputDim, const size_t outputDim)
{
    cgStdFun_ = f;
    outputDim_ = outputDim;
    inputDim_ = inputDim;
    if (outputDim_ > 0 && inputDim_ > 0)
    {
        recordCg();
        compiled_ = false;
        libName_ = "";
        dynamicLib_ = nullptr;
        model_ = nullptr;
    }
}

template <int IN_DIM, int OUT_DIM>
auto DerivativesCppadJIT<IN_DIM, OUT_DIM>::clone() const -> DerivativesCppadJIT*
{
    return new DerivativesCppadJIT<IN_DIM, OUT_DIM>(*this);
}

template <int IN_DIM, int OUT_DIM>
auto DerivativesCppadJIT<IN_DIM, OUT_DIM>::forwardZero(const Eigen::VectorXd& x) -> OUT_TYPE_D
{
    if (compiled_)
    {
        assert(model_->isForwardZeroAvailable() == true);
        return model_->ForwardZero(x);
    }
    else
    {
        throw std::runtime_error("Error: Compile the library first by calling compileJIT(..)");
    }
}

template <int IN_DIM, int OUT_DIM>
auto DerivativesCppadJIT<IN_DIM, OUT_DIM>::jacobian(const Eigen::VectorXd& x) -> JAC_TYPE_D
{
    if (outputDim_ <= 0)
        throw std::runtime_error("Outdim dim smaller 0; Define output dim in DerivativesCppad constructor");


    Eigen::VectorXd jac;
    if (compiled_)
    {
        assert(model_->isJacobianAvailable() == true);
        jac = model_->Jacobian(x);
        JAC_TYPE_D out(outputDim_, x.rows());
        out = JAC_TYPE_ROW_MAJOR::Map(jac.data(), outputDim_, x.rows());
        return out;
    }
    else
        throw std::runtime_error("Error: Compile the library first by calling compileJIT(..)");
}

template <int IN_DIM, int OUT_DIM>
void DerivativesCppadJIT<IN_DIM, OUT_DIM>::sparseJacobian(const Eigen::VectorXd& x,
    Eigen::VectorXd& jac,
    Eigen::VectorXi& iRow,
    Eigen::VectorXi& jCol)
{
    if (outputDim_ <= 0)
        throw std::runtime_error("Outdim dim smaller 0; Define output dim in DerivativesCppad constructor");

    if (compiled_)
    {
        assert(model_->isSparseJacobianAvailable() == true);
        std::vector<double> input(x.data(), x.data() + x.rows() * x.cols());
        std::vector<double> output;
        model_->SparseJacobian(input, output, sparsityRowsJacobian_, sparsityColsJacobian_);
        jac = Eigen::Map<Eigen::VectorXd>(output.data(), output.size(), 1);
        iRow = sparsityRowsJacobianEigen_;
        jCol = sparsityColsJacobianEigen_;
    }
    else
        throw std::runtime_error("Error: Compile the library first by calling compileJIT(..)");
}

template <int IN_DIM, int OUT_DIM>
Eigen::VectorXd DerivativesCppadJIT<IN_DIM, OUT_DIM>::sparseJacobianValues(const Eigen::VectorXd& x)
{
    if (outputDim_ <= 0)
        throw std::runtime_error("Outdim dim smaller 0; Define output dim in DerivativesCppad constructor");

    if (compiled_)
    {
        assert(model_->isSparseJacobianAvailable() == true);
        std::vector<double> input(x.data(), x.data() + x.rows() * x.cols());
        std::vector<double> output;
        model_->SparseJacobian(input, output, sparsityRowsJacobian_, sparsityColsJacobian_);
        return Eigen::Map<Eigen::VectorXd>(output.data(), output.size(), 1);
    }
    else
        throw std::runtime_error("Error: Compile the library first by calling compileJIT(..)");
}

template <int IN_DIM, int OUT_DIM>
auto DerivativesCppadJIT<IN_DIM, OUT_DIM>::hessian(const Eigen::VectorXd& x, const Eigen::VectorXd& lambda)
    -> HES_TYPE_D
{
    if (outputDim_ <= 0)
        throw std::runtime_error("Outdim dim smaller 0; Define output dim in DerivativesCppad constructor");

    if (compiled_)
    {
        assert(model_->isHessianAvailable() == true);
        Eigen::VectorXd hessian = model_->Hessian(x, lambda);
        HES_TYPE_D out(x.rows(), x.rows());
        out = HES_TYPE_ROW_MAJOR::Map(hessian.data(), x.rows(), x.rows());
        return out;
    }
    else
    {
        throw std::runtime_error("Error: Compile the library first by calling compileJIT(..)");
    }
}

template <int IN_DIM, int OUT_DIM>
void DerivativesCppadJIT<IN_DIM, OUT_DIM>::sparseHessian(const Eigen::VectorXd& x,
    const Eigen::VectorXd& lambda,
    Eigen::VectorXd& hes,
    Eigen::VectorXi& iRow,
    Eigen::VectorXi& jCol)
{
    if (outputDim_ <= 0)
        throw std::runtime_error("Outdim dim smaller 0; Define output dim in DerivativesCppad constructor");

    if (compiled_)
    {
        assert(model_->isSparseJacobianAvailable() == true);
        std::vector<double> input(x.data(), x.data() + x.rows() * x.cols());
        std::vector<double> inputLambda(lambda.data(), lambda.data() + lambda.rows() * lambda.cols());
        std::vector<double> output;
        model_->SparseHessian(input, inputLambda, output, sparsityRowsHessian_, sparsityColsHessian_);
        hes = Eigen::Map<Eigen::VectorXd>(output.data(), output.size(), 1);
        iRow = sparsityRowsHessianEigen_;
        jCol = sparsityColsHessianEigen_;
    }
    else
        throw std::runtime_error("Error: Compile the library first by calling compileJIT(..)");
}

template <int IN_DIM, int OUT_DIM>
Eigen::VectorXd DerivativesCppadJIT<IN_DIM, OUT_DIM>::sparseHessianValues(const Eigen::VectorXd& x,
    const Eigen::VectorXd& lambda)
{
    if (outputDim_ <= 0)
        throw std::runtime_error("Outdim dim smaller 0; Define output dim in DerivativesCppad constructor");

    if (compiled_)
    {
        assert(model_->isSparseHessianAvailable() == true);
        std::vector<double> input(x.data(), x.data() + x.rows() * x.cols());
        std::vector<double> inputLambda(lambda.data(), lambda.data() + lambda.rows() * lambda.cols());
        std::vector<double> output;
        model_->SparseHessian(input, inputLambda, output, sparsityRowsHessian_, sparsityColsHessian_);
        return Eigen::Map<Eigen::VectorXd>(output.data(), output.size(), 1);
    }
    else
        throw std::runtime_error("Error: Compile the library first by calling compileJIT(..)");
}

template <int IN_DIM, int OUT_DIM>
Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> DerivativesCppadJIT<IN_DIM, OUT_DIM>::getSparsityPatternJacobian()
{
    assert(model_->isJacobianSparsityAvailable() == true);

    std::vector<bool> sparsityVec = model_->JacobianSparsityBool();
    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> sparsityMat(outputDim_, inputDim_);

    assert((int)(sparsityVec.size()) == outputDim_ * inputDim_);
    for (int row = 0; row < outputDim_; ++row)
        for (int col = 0; col < inputDim_; ++col)
            sparsityMat(row, col) = sparsityVec[col + row * inputDim_];

    return sparsityMat;
}

template <int IN_DIM, int OUT_DIM>
Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> DerivativesCppadJIT<IN_DIM, OUT_DIM>::getSparsityPatternHessian()
{
    assert(model_->isHessianSparsityAvailable() == true);

    std::vector<bool> sparsityVec = model_->HessianSparsityBool();
    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> sparsityMat(inputDim_, inputDim_);

    assert(sparsityVec.size() == inputDim_ * inputDim_);
    for (int row = 0; row < inputDim_; ++row)
        for (int col = 0; col < inputDim_; ++col)
        {
            sparsityMat(row, col) = sparsityVec[col + row * inputDim_];
        }

    return sparsityMat;
}

template <int IN_DIM, int OUT_DIM>
void DerivativesCppadJIT<IN_DIM, OUT_DIM>::getSparsityPatternJacobian(Eigen::VectorXi& rows, Eigen::VectorXi& columns)
{
    assert(model_->isJacobianSparsityAvailable() == true);

    rows = sparsityRowsJacobianEigen_;
    columns = sparsityColsJacobianEigen_;
}

template <int IN_DIM, int OUT_DIM>
size_t DerivativesCppadJIT<IN_DIM, OUT_DIM>::getNumNonZerosJacobian()
{
    assert(model_->isJacobianSparsityAvailable() == true);
    return sparsityRowsJacobian_.size();
}

template <int IN_DIM, int OUT_DIM>
size_t DerivativesCppadJIT<IN_DIM, OUT_DIM>::getNumNonZerosHessian()
{
    assert(model_->isJacobianSparsityAvailable() == true);
    return sparsityRowsHessian_.size();
}

template <int IN_DIM, int OUT_DIM>
void DerivativesCppadJIT<IN_DIM, OUT_DIM>::getSparsityPatternHessian(Eigen::VectorXi& rows, Eigen::VectorXi& columns)
{
    assert(model_->isHessianSparsityAvailable() == true);

    rows = sparsityRowsHessianEigen_;
    columns = sparsityColsHessianEigen_;
}

template <int IN_DIM, int OUT_DIM>
void DerivativesCppadJIT<IN_DIM, OUT_DIM>::compileJIT(const DerivativesCppadSettings& settings,
    const std::string& libName,
    bool verbose)
{
    if (compiled_)
        return;

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);

    // assigning a unique identifier to the library in order to avoid race conditions in JIT
    std::string uniqueID =
        std::to_string(std::hash<std::thread::id>()(std::this_thread::get_id())) + "_" + std::to_string(ts.tv_nsec);

    libName_ = libName + uniqueID;

    CppAD::cg::ModelCSourceGen<double> cgen(cgCppadFun_, libName_);

    cgen.setMultiThreading(settings.multiThreading_);
    cgen.setCreateForwardZero(settings.createForwardZero_);
    cgen.setCreateForwardOne(settings.createForwardOne_);
    cgen.setCreateReverseOne(settings.createReverseOne_);
    cgen.setCreateReverseTwo(settings.createReverseTwo_);
    cgen.setCreateJacobian(settings.createJacobian_);
    cgen.setCreateSparseJacobian(settings.createSparseJacobian_);
    cgen.setCreateHessian(settings.createHessian_);
    cgen.setCreateSparseHessian(settings.createSparseHessian_);
    cgen.setMaxAssignmentsPerFunc(settings.maxAssignements_);

    CppAD::cg::ModelLibraryCSourceGen<double> libcgen(cgen);

    if (settings.useDynamicLibrary_)
    {
        std::string tempDir = "cppad_temp" + uniqueID;
        if (verbose)
        {
            std::cout << "DerivativesCppadJIT: starting to compile dynamic library" << libName_ << std::endl;
            std::cout << "DerivativesCppadJIT: in temporary directory " << tempDir << std::endl;
        }

        // compile a dynamic library
        CppAD::cg::DynamicModelLibraryProcessor<double> p(libcgen, libName_);

        if (settings.compiler_ == DerivativesCppadSettings::GCC)
        {
            CppAD::cg::GccCompiler<double> compiler;
            compiler.setTemporaryFolder(tempDir);
            dynamicLib_ = std::shared_ptr<CppAD::cg::DynamicLib<double>>(p.createDynamicLibrary(compiler));
        }
        else if (settings.compiler_ == DerivativesCppadSettings::CLANG)
        {
            CppAD::cg::ClangCompiler<double> compiler;
            compiler.setTemporaryFolder(tempDir);
            dynamicLib_ = std::shared_ptr<CppAD::cg::DynamicLib<double>>(p.createDynamicLibrary(compiler));
        }
        else
            throw std::runtime_error("Unknown compiler type for dynamic library, support only gcc and clang.");

        // extract model
        model_ = std::shared_ptr<CppAD::cg::GenericModel<double>>(dynamicLib_->model(libName_));
    }
    else  // use regular JIT
    {
        if (verbose)
        {
            std::cout << "DerivativesCppadJIT: starting to compile with LLVM library " << libName_ << std::endl;
        }
#ifdef LLVM_VERSION_MAJOR
        // JIT compile source code
        CppAD::cg::LlvmModelLibraryProcessor<double> p(libcgen);
        llvmModelLib_ = p.create();

        //extract model
        model_ = std::shared_ptr<CppAD::cg::GenericModel<double>>(llvmModelLib_->model(libName_));
#else
        throw std::runtime_error("DerivativesCppadJIT: LLVM not installed.");
#endif
    }


    if (settings.generateSourceCode_)
    {
        if (verbose)
        {
            std::cout << "DerivativesCppadJIT: generating source-code... " << libName_ << std::endl;
        }
        CppAD::cg::SaveFilesModelLibraryProcessor<double> p2(libcgen);
        p2.saveSources();
    }

    compiled_ = true;

    if (model_->isJacobianSparsityAvailable())
    {
        if (verbose)
            std::cout << "DerivativesCppadJIT: updating Jacobian sparsity pattern" << std::endl;

        model_->JacobianSparsity(sparsityRowsJacobian_, sparsityColsJacobian_);
        sparsityRowsJacobianEigen_.resize(sparsityRowsJacobian_.size());  //rowsTmp.resize(rowsVec.size());
        sparsityColsJacobianEigen_.resize(sparsityColsJacobian_.size());  //colsTmp.resize(rowsVec.size());

        Eigen::Matrix<size_t, Eigen::Dynamic, 1> rowsSizeT;
        rowsSizeT.resize(sparsityRowsJacobian_.size());
        Eigen::Matrix<size_t, Eigen::Dynamic, 1> colsSizeT;
        colsSizeT.resize(sparsityColsJacobian_.size());

        rowsSizeT = Eigen::Map<Eigen::Matrix<size_t, Eigen::Dynamic, 1>>(
            sparsityRowsJacobian_.data(), sparsityRowsJacobian_.size(), 1);
        colsSizeT = Eigen::Map<Eigen::Matrix<size_t, Eigen::Dynamic, 1>>(
            sparsityColsJacobian_.data(), sparsityColsJacobian_.size(), 1);

        sparsityRowsJacobianEigen_ = rowsSizeT.cast<int>();
        sparsityColsJacobianEigen_ = colsSizeT.cast<int>();
    }

    if (model_->isHessianSparsityAvailable())
    {
        if (verbose)
            std::cout << "DerivativesCppadJIT: updating Hessian sparsity pattern" << std::endl;

        model_->HessianSparsity(sparsityRowsHessian_, sparsityColsHessian_);
        sparsityRowsHessianEigen_.resize(sparsityRowsHessian_.size());  //rowsTmp.resize(rowsVec.size());
        sparsityColsHessianEigen_.resize(sparsityColsHessian_.size());  //colsTmp.resize(rowsVec.size());

        Eigen::Matrix<size_t, Eigen::Dynamic, 1> rowsSizeT;
        rowsSizeT.resize(sparsityRowsHessian_.size());
        Eigen::Matrix<size_t, Eigen::Dynamic, 1> colsSizeT;
        colsSizeT.resize(sparsityColsHessian_.size());

        rowsSizeT = Eigen::Map<Eigen::Matrix<size_t, Eigen::Dynamic, 1>>(
            sparsityRowsHessian_.data(), sparsityRowsHessian_.size(), 1);
        colsSizeT = Eigen::Map<Eigen::Matrix<size_t, Eigen::Dynamic, 1>>(
            sparsityColsHessian_.data(), sparsityColsHessian_.size(), 1);

        sparsityRowsHessianEigen_ = rowsSizeT.cast<int>();
        sparsityColsHessianEigen_ = colsSizeT.cast<int>();
    }

    if (verbose)
        std::cout << "DerivativesCppadJIT: compileJIT() completed." << std::endl;
}

template <int IN_DIM, int OUT_DIM>
auto DerivativesCppadJIT<IN_DIM, OUT_DIM>::getDynamicLib() -> const std::shared_ptr<CppAD::cg::DynamicLib<double>>
{
    if (dynamicLib_)
        return dynamicLib_;
    else
        throw std::runtime_error("DerivativesCppADJIT: dynamic lib not compiled.");
}

#ifdef LLVM_VERSION_MAJOR
template <int IN_DIM, int OUT_DIM>
auto DerivativesCppadJIT<IN_DIM, OUT_DIM>::getLlvmLib() -> const std::shared_ptr<CppAD::cg::LlvmModelLibrary<double>>
{
    if (llvmModelLib_)
        return llvmModelLib_;
    else
        throw std::runtime_error("DerivativesCppADJIT: llvm library not available.");
}
#endif

template <int IN_DIM, int OUT_DIM>
void DerivativesCppadJIT<IN_DIM, OUT_DIM>::recordCg()
{
    // input vector, needs to be dynamic size
    Eigen::Matrix<CG_SCALAR, Eigen::Dynamic, 1> x(inputDim_);

    // declare x as independent
    CppAD::Independent(x);

    // output vector, needs to be dynamic size
    Eigen::Matrix<CG_SCALAR, Eigen::Dynamic, 1> y(outputDim_);

    y = cgStdFun_(x);

    // store operation sequence in f: x -> y and stop recording
    CppAD::ADFun<CG_VALUE_TYPE> fCodeGen(x, y);

    fCodeGen.optimize();

    cgCppadFun_ = fCodeGen;
}


} /* namespace core */
} /* namespace ct */

#endif