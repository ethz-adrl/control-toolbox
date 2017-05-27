#ifndef CPPAD_CG_LINUX_DYNAMICLIB_INCLUDED
#define CPPAD_CG_LINUX_DYNAMICLIB_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2012 Ciengis
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
#if CPPAD_CG_SYSTEM_LINUX

#include <typeinfo>
#include <dlfcn.h>

namespace CppAD {
namespace cg {

/**
 * Useful class to call the compiled source code in a dynamic library.
 * For the Linux Operating System only.
 * 
 * @author Joao Leal
 */
template<class Base>
class LinuxDynamicLib : public DynamicLib<Base> {
protected:
    const std::string _dynLibName;
    /// the dynamic library handler
    void* _dynLibHandle;
    std::set<LinuxDynamicLibModel<Base>*> _models;
public:

    LinuxDynamicLib(const std::string& dynLibName,
                    int dlOpenMode = RTLD_NOW) :
        _dynLibName(dynLibName),
        _dynLibHandle(nullptr) {

        std::string path;
        if (dynLibName[0] == '/') {
            path = dynLibName; // absolute path
        } else if (!(dynLibName[0] == '.' && dynLibName[1] == '/') &&
                !(dynLibName[0] == '.' && dynLibName[1] == '.')) {
            path = "./" + dynLibName; // relative path
        } else {
            path = dynLibName;
        }

        // load the dynamic library
        _dynLibHandle = dlopen(path.c_str(), dlOpenMode);
        CPPADCG_ASSERT_KNOWN(_dynLibHandle != nullptr, ("Failed to dynamically load library '" + dynLibName + "': " + dlerror()).c_str());

        // validate the dynamic library
        this->validate();
    }

    LinuxDynamicLib(const LinuxDynamicLib&) = delete;
    LinuxDynamicLib& operator=(const LinuxDynamicLib&) = delete;

    virtual LinuxDynamicLibModel<Base>* model(const std::string& modelName) override {
        std::set<std::string>::const_iterator it = this->_modelNames.find(modelName);
        if (it == this->_modelNames.end()) {
            return nullptr;
        }
        LinuxDynamicLibModel<Base>* m = new LinuxDynamicLibModel<Base> (this, modelName);
        _models.insert(m);
        return m;
    }

    virtual void* loadFunction(const std::string& functionName, bool required = true) override {
        void* functor = dlsym(_dynLibHandle, functionName.c_str());

        if (required) {
            char *err = dlerror();
            if (err != nullptr)
                throw CGException("Failed to load function '", functionName, "': ", err);
        }

        return functor;
    }

    virtual ~LinuxDynamicLib() {
        for (LinuxDynamicLibModel<Base>* model : _models) {
            model->modelLibraryClosed();
        }

        if (_dynLibHandle != nullptr) {
            if(this->_onClose != nullptr) {
                (*this->_onClose)();
            }

            dlclose(_dynLibHandle);
            _dynLibHandle = nullptr;
        }
    }

protected:

    virtual void destroyed(LinuxDynamicLibModel<Base>* model) {
        _models.erase(model);
    }

    friend class LinuxDynamicLibModel<Base>;

};

} // END cg namespace
} // END CppAD namespace

#endif
#endif