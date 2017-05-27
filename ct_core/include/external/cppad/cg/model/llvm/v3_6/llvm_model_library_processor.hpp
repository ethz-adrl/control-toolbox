#ifndef CPPAD_CG_LLVM_MODEL_LIBRARY_PROCESSOR_INCLUDED
#define CPPAD_CG_LLVM_MODEL_LIBRARY_PROCESSOR_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2015 Ciengis
 *
 *  CppADCodeGen is distributed under multiple licenses:
 *
 *   - Eclipse Public License Version 1.0 (EPL1), and
 *   - GNU General Public License Version 3 (GPL3).
 *
 *  EPL1 terms and conditions can be found in the file "epl-v10.txt", while
 *  terms and conditions for the GPL3 can be found in the file "gpl3.txt".
 * ----------------------------------------------------------------------------
 * Author: Joao Leal
 */

#include <cppad/cg/model/llvm/llvm_base_model_library_processor.hpp>

namespace CppAD {
namespace cg {

/**
 * Useful class for generating a JIT evaluated model library.
 * 
 * @author Joao Leal
 */
template<class Base>
class LlvmModelLibraryProcessor : public LlvmBaseModelLibraryProcessor<Base> {
protected:
    std::vector<std::string> _includePaths;
    std::unique_ptr<llvm::Linker> _linker;
    std::unique_ptr<llvm::LLVMContext> _context;
    std::unique_ptr<llvm::Module> _module;
public:

    /**
     * 
     * @param modelLibraryHelper
     */
    LlvmModelLibraryProcessor(ModelLibraryCSourceGen<Base>& modelLibraryHelper) :
            LlvmBaseModelLibraryProcessor<Base>(modelLibraryHelper) {
    }

    virtual ~LlvmModelLibraryProcessor() {
    }

    inline void setIncludePaths(const std::vector<std::string>& includePaths) {
        _includePaths = includePaths;
    }

    inline const std::vector<std::string>& getIncludePaths() const {
        return _includePaths;
    }

    LlvmModelLibrary<Base>* create() {
        // backup output format so that it can be restored
        OStreamConfigRestore coutb(std::cout);

        _linker.reset(nullptr);

        this->modelLibraryHelper_->startingJob("", JobTimer::JIT_MODEL_LIBRARY);

        llvm::InitializeAllTargets();
        llvm::InitializeAllAsmPrinters();

        _context.reset(new llvm::LLVMContext());

        const std::map<std::string, ModelCSourceGen<Base>*>& models = this->modelLibraryHelper_->getModels();
        for (const auto& p : models) {
            const std::map<std::string, std::string>& modelSources = this->getSources(*p.second);
            createLlvmModules(modelSources);
        }

        const std::map<std::string, std::string>& sources = this->getLibrarySources();
        createLlvmModules(sources);

        const std::map<std::string, std::string>& customSource = this->modelLibraryHelper_->getCustomSources();
        createLlvmModules(customSource);

        llvm::InitializeNativeTarget();

        LlvmModelLibrary3_6<Base>* lib = new LlvmModelLibrary3_6<Base>(_module.release(), _context.release());

        this->modelLibraryHelper_->finishedJob();

        return lib;
    }

    static inline LlvmModelLibrary<Base>* create(ModelLibraryCSourceGen<Base>& modelLibraryHelper) {
        LlvmModelLibraryProcessor<Base> p(modelLibraryHelper);
        return p.create();
    }

protected:

    virtual void createLlvmModules(const std::map<std::string, std::string>& sources) {
        for (const auto& p : sources) {
            createLlvmModule(p.first, p.second);
        }
    }

    virtual void createLlvmModule(const std::string& filename,
                                  const std::string& source) {
        using namespace llvm;
        using namespace clang;

        static const char* argv [] = {"program", "-Wall", "-x", "c", "string-input"};  // -Wall or -v flag is required to avoid an error inside createInvocationFromCommandLine()
        static const int argc = sizeof (argv) / sizeof (argv[0]);

        IntrusiveRefCntPtr<DiagnosticOptions> diagOpts = new DiagnosticOptions();
        TextDiagnosticPrinter* diagClient = new TextDiagnosticPrinter(llvm::errs(), &*diagOpts); // will be owned by diags
        IntrusiveRefCntPtr<DiagnosticIDs> diagID(new DiagnosticIDs());
        IntrusiveRefCntPtr<DiagnosticsEngine> diags(new DiagnosticsEngine(diagID, &*diagOpts, diagClient));

        ArrayRef<const char*> args(argv + 1, // skip program name
                                   argc - 1);
        std::unique_ptr<CompilerInvocation> invocation(createInvocationFromCommandLine(args, diags));
        if (invocation.get() == nullptr)
            throw CGException("Failed to create compiler invocation");
        CompilerInvocation::setLangDefaults(*invocation->getLangOpts(), IK_C,
                                            LangStandard::lang_unspecified);
        invocation->getFrontendOpts().DisableFree = false; // make sure we free memory (by default it does not)

        // Create a compiler instance to handle the actual work.
        CompilerInstance compiler;
        compiler.setInvocation(invocation.release());


        // Create the compilers actual diagnostics engine.
        compiler.createDiagnostics(); //compiler.createDiagnostics(argc, const_cast<char**> (argv));
        if (!compiler.hasDiagnostics())
            throw CGException("No diagnostics");
#if 0
        compiler.createFileManager();
        compiler.createSourceManager(compiler.getFileManager());
        std::shared_ptr<clang::TargetOptions> pto = std::make_shared<clang::TargetOptions>();
        pto->Triple = llvm::sys::getDefaultTargetTriple();
        clang::TargetInfo *pti = clang::TargetInfo::CreateTargetInfo(compiler.getDiagnostics(), pto);
        compiler.setTarget(pti);
        compiler.createPreprocessor(clang::TU_Complete);
#endif

        // Create memory buffer with source text
        std::unique_ptr<llvm::MemoryBuffer> buffer = llvm::MemoryBuffer::getMemBufferCopy(source, "SIMPLE_BUFFER");
        if (buffer.get() == nullptr)
            throw CGException("Failed to create memory buffer");

        // Remap auxiliary name "string-input" to memory buffer
        PreprocessorOptions& po = compiler.getInvocation().getPreprocessorOpts();
        po.addRemappedFile("string-input", buffer.release());

        HeaderSearchOptions& hso = compiler.getInvocation().getHeaderSearchOpts();
        std::string iClangHeaders = this->findInternalClangCHeaders("3.6", hso.ResourceDir);
        if(!iClangHeaders.empty()) {
            hso.AddPath(llvm::StringRef(iClangHeaders), clang::frontend::Angled, false, false);
        }

        for (size_t s = 0; s < _includePaths.size(); s++)
            hso.AddPath(llvm::StringRef(_includePaths[s]), clang::frontend::Angled, false, false);

        // Create and execute the frontend to generate an LLVM bitcode module.
        clang::EmitLLVMOnlyAction action(_context.get());
        if (!compiler.ExecuteAction(action))
            throw CGException("Failed to emit LLVM bitcode");

        std::unique_ptr<llvm::Module> module = action.takeModule();
        if (module.get() == nullptr)
            throw CGException("No module");

        if (_linker.get() == nullptr) {
            _module.reset(module.release());
            _linker.reset(new llvm::Linker(_module.get()));
        } else {
            if (_linker->linkInModule(module.release())) {
                throw CGException("LLVM failed to link module");
            }
        }

        // NO delete module;
        // NO delete invocation;
        //llvm::llvm_shutdown();
    }

};

} // END cg namespace
} // END CppAD namespace

#endif