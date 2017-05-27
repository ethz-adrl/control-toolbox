#ifndef CPPAD_CG_LLVM_MODEL_LIBRARY_INCLUDED
#define CPPAD_CG_LLVM_MODEL_LIBRARY_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2013 Ciengis
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

namespace CppAD {
namespace cg {

template<class Base>
class LlvmModel;

/**
 * Abstract class used to load JIT'ed models by LLVM
 * 
 * @author Joao Leal
 */
template<class Base>
class LlvmModelLibrary : public FunctorModelLibrary<Base> {
protected:
    std::set<LlvmModel<Base>*> _models;
public:
    virtual LlvmModel<Base>* model(const std::string& modelName) override {
        typename std::set<std::string>::const_iterator it = this->_modelNames.find(modelName);
        if (it == this->_modelNames.end()) {
            return nullptr;
        }
        LlvmModel<Base>* m = new LlvmModel<Base> (this, modelName);
        _models.insert(m);
        return m;
    }

    inline virtual ~LlvmModelLibrary() {
        // do not call clean-up here
        // cleanUp() must be called by the subclass (before destruction of the execution engine...)
    }

protected:
    inline LlvmModelLibrary() {
    }

    inline void cleanUp() {
        for (LlvmModel<Base>* model : _models) {
            model->modelLibraryClosed();
        }

        if(this->_onClose != nullptr) {
            (*this->_onClose)();
            this->_onClose = nullptr;
        }
    }

    virtual void destroyed(LlvmModel<Base>* model) {
        _models.erase(model);
    }

    friend class LlvmModel<Base>;
};

} // END cg namespace
} // END CppAD namespace

#endif