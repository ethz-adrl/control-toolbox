#ifndef CPPAD_CG_LANGUAGE_C_INCLUDED
#define CPPAD_CG_LANGUAGE_C_INCLUDED
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

#define CPPAD_CG_C_LANG_FUNCNAME(fn) \
inline virtual const std::string& fn ## FuncName() {\
    static const std::string name(#fn);\
    return name;\
}

namespace CppAD {
namespace cg {

/**
 * Generates code for the C language
 * 
 * @author Joao Leal
 */
template<class Base>
class LanguageC : public Language<Base> {
public:
    typedef OperationNode<Base> Node;
    typedef Argument<Base> Arg;
public:
    static const std::string U_INDEX_TYPE;
    static const std::string ATOMICFUN_STRUCT_DEFINITION;
protected:
    static const std::string _C_COMP_OP_LT;
    static const std::string _C_COMP_OP_LE;
    static const std::string _C_COMP_OP_EQ;
    static const std::string _C_COMP_OP_GE;
    static const std::string _C_COMP_OP_GT;
    static const std::string _C_COMP_OP_NE;
    static const std::string _C_STATIC_INDEX_ARRAY;
    static const std::string _C_SPARSE_INDEX_ARRAY;
    static const std::string _ATOMIC_TX;
    static const std::string _ATOMIC_TY;
    static const std::string _ATOMIC_PX;
    static const std::string _ATOMIC_PY;
private:
    class AtomicFuncArray; //forward declaration
protected:
    // the type name of the Base class (e.g. "double")
    const std::string _baseTypeName;
    // spaces for 1 level indentation
    const std::string _spaces;
    // information from the code handler (not owned)
    LanguageGenerationData<Base>* _info;
    // current indentation
    std::string _indentation;
    // variable name used for the inlet variable
    std::string _inArgName;
    // variable name used for the outlet variable
    std::string _outArgName;
    // variable name used for the atomic functions array
    std::string _atomicArgName;
    // output stream for the generated source code
    std::ostringstream _code;
    // creates the variable names
    VariableNameGenerator<Base>* _nameGen;
    // auxiliary string stream
    std::ostringstream _ss;
    //
    size_t _independentSize;
    //
    size_t _minTemporaryVarID;
    // maps the variable IDs to the their position in the dependent vector
    // (some IDs may be the same as the independent variables when dep = indep)
    std::map<size_t, size_t> _dependentIDs;
    // the dependent variable vector
    const ArrayWrapper<CG<Base> >* _dependent;
    // the temporary variables that may require a declaration
    std::map<size_t, Node*> _temporary;
    // the operator used for assignment of dependent variables
    std::string _depAssignOperation;
    // whether or not to ignore assignment of constant zero values to dependent variables
    bool _ignoreZeroDepAssign;
    // the name of the function to be created (if the string is empty no function is created)
    std::string _functionName;
    // the arguments provided to local functions called by the main function
    std::string _localFunctionArguments;
    // the maximum number of assignment (~lines) per local function
    size_t _maxAssigmentsPerFunction;
    //
    std::map<std::string, std::string>* _sources;
    // the values in the temporary array
    std::vector<const Arg*> _tmpArrayValues;
    // the values in the temporary sparse array
    std::vector<const Arg*> _tmpSparseArrayValues;
    // the current state of Array structures used by atomic functions
    std::map<std::string, AtomicFuncArray> _atomicFuncArrays;
    // indexes defined as function arguments
    std::vector<const Node*> _funcArgIndexes;
    std::vector<const LoopStartOperationNode<Base>*> _currentLoops;
    // the maximum precision used to print values
    size_t _parameterPrecision;
private:
    std::string funcArgDcl_;
    std::string localFuncArgDcl_;
    std::string localFuncArgs_;
    std::string auxArrayName_;

public:

    /**
     * Creates a C language source code generator
     * 
     * @param varTypeName variable data type (e.g. double)
     * @param spaces number of spaces for indentations
     */
    LanguageC(const std::string& varTypeName,
              size_t spaces = 3) :
        _baseTypeName(varTypeName),
        _spaces(spaces, ' '),
        _info(nullptr),
        _inArgName("in"),
        _outArgName("out"),
        _atomicArgName("atomicFun"),
        _nameGen(nullptr),
        _independentSize(0), // not really required (but it avoids warnings)
        _minTemporaryVarID(0), // not really required (but it avoids warnings)
        _dependent(nullptr),
        _depAssignOperation("="),
        _ignoreZeroDepAssign(false),
        _maxAssigmentsPerFunction(0),
        _sources(nullptr),
        _parameterPrecision(std::numeric_limits<Base>::digits10) {
    }

    inline const std::string& getArgumentIn() const {
        return _inArgName;
    }

    inline void setArgumentIn(const std::string& inArgName) {
        _inArgName = inArgName;
    }

    inline const std::string& getArgumentOut() const {
        return _outArgName;
    }

    inline void setArgumentOut(const std::string& outArgName) {
        _outArgName = outArgName;
    }

    inline const std::string& getArgumentAtomic() const {
        return _atomicArgName;
    }

    inline void setArgumentAtomic(const std::string& atomicArgName) {
        _atomicArgName = atomicArgName;
    }

    inline const std::string& getDependentAssignOperation() const {
        return _depAssignOperation;
    }

    inline void setDependentAssignOperation(const std::string& depAssignOperation) {
        _depAssignOperation = depAssignOperation;
    }

    inline bool isIgnoreZeroDepAssign() const {
        return _ignoreZeroDepAssign;
    }

    inline void setIgnoreZeroDepAssign(bool ignore) {
        _ignoreZeroDepAssign = ignore;
    }

    virtual void setGenerateFunction(const std::string& functionName) {
        _functionName = functionName;
    }

    virtual void setFunctionIndexArgument(const Node& funcArgIndex) {
        _funcArgIndexes.resize(1);
        _funcArgIndexes[0] = &funcArgIndex;
    }

    virtual void setFunctionIndexArguments(const std::vector<const Node*>& funcArgIndexes) {
        _funcArgIndexes = funcArgIndexes;
    }

    virtual const std::vector<const Node*>& getFunctionIndexArguments() const {
        return _funcArgIndexes;
    }

    /**
     * Provides the maximum precision used to print constant values in the
     * generated source code
     * 
     * @return the maximum number of digits
     */
    virtual size_t getParameterPrecision() const {
        return _parameterPrecision;
    }

    /**
     * Defines the maximum precision used to print constant values in the
     * generated source code
     * 
     * @param p the maximum number of digits
     */
    virtual void setParameterPrecision(size_t p) {
        _parameterPrecision = p;
    }

    virtual void setMaxAssigmentsPerFunction(size_t maxAssigmentsPerFunction,
                                             std::map<std::string, std::string>* sources) {
        _maxAssigmentsPerFunction = maxAssigmentsPerFunction;
        _sources = sources;
    }

    inline std::string generateTemporaryVariableDeclaration(bool isWrapperFunction,
                                                            bool zeroArrayDependents,
                                                            const std::vector<int>& atomicMaxForward,
                                                            const std::vector<int>& atomicMaxReverse) {
        int maxForward = -1;
        if (!atomicMaxForward.empty())
            maxForward = *std::max_element(atomicMaxForward.begin(), atomicMaxForward.end());

        int maxReverse = -1;
        if (!atomicMaxReverse.empty())
            maxReverse = *std::max_element(atomicMaxReverse.begin(), atomicMaxReverse.end());

        return generateTemporaryVariableDeclaration(isWrapperFunction, zeroArrayDependents,
                                                    maxForward, maxReverse);
    }

    /**
     * Declares temporary variables used by a function.
     * 
     * @param isWrapperFunction true if the declarations are for a wrapper
     *                               function which calls other functions
     *                               where the actual work is performed
     * @param zeroArrayDependents  whether or not the dependent variables
     *                             should be set to zero before executing 
     *                             the operation graph
     * @param maxForwardOrder the maximum order of forward mode calls to 
     *                        atomic functions
     * @param maxReverseOrder the maximum order of reverse mode calls to 
     *                        atomic functions
     * @return the string with the declarations for the temporary variables
     */
    virtual std::string generateTemporaryVariableDeclaration(bool isWrapperFunction = false,
                                                             bool zeroArrayDependents = false,
                                                             int maxForwardOrder = -1,
                                                             int maxReverseOrder = -1) {
        CPPADCG_ASSERT_UNKNOWN(_nameGen != nullptr);

        // declare variables
        const std::vector<FuncArgument>& tmpArg = _nameGen->getTemporary();

        CPPADCG_ASSERT_KNOWN(tmpArg.size() == 3,
                             "There must be two temporary variables");

        _ss << _spaces << "// auxiliary variables\n";
        /**
         * temporary variables
         */
        if (tmpArg[0].array) {
            size_t size = _nameGen->getMaxTemporaryVariableID() + 1 - _nameGen->getMinTemporaryVariableID();
            if (size > 0 || isWrapperFunction) {
                _ss << _spaces << _baseTypeName << " " << tmpArg[0].name << "[" << size << "];\n";
            }
        } else if (_temporary.size() > 0) {
            for (const std::pair<size_t, Node*>& p : _temporary) {
                Node* var = p.second;
                if (var->getName() == nullptr) {
                    var->setName(_nameGen->generateTemporary(*var, getVariableID(*var)));
                }
            }

            Node* var1 = _temporary.begin()->second;
            const std::string& varName1 = *var1->getName();
            _ss << _spaces << _baseTypeName << " " << varName1;

            typename std::map<size_t, Node*>::const_iterator it = _temporary.begin();
            for (it++; it != _temporary.end(); ++it) {
                _ss << ", " << *it->second->getName();
            }
            _ss << ";\n";
        }

        /**
         * temporary array variables
         */
        size_t arraySize = _nameGen->getMaxTemporaryArrayVariableID();
        if (arraySize > 0 || isWrapperFunction) {
            _ss << _spaces << _baseTypeName << " " << tmpArg[1].name << "[" << arraySize << "];\n";
        }

        /**
         * temporary sparse array variables
         */
        size_t sArraySize = _nameGen->getMaxTemporarySparseArrayVariableID();
        if (sArraySize > 0 || isWrapperFunction) {
            _ss << _spaces << _baseTypeName << " " << tmpArg[2].name << "[" << sArraySize << "];\n";
            _ss << _spaces << U_INDEX_TYPE << " " << _C_SPARSE_INDEX_ARRAY << "[" << sArraySize << "];\n";
        }

        if (!isWrapperFunction) {
            generateArrayContainersDeclaration(_ss, maxForwardOrder, maxReverseOrder);
        }

        //
        if (!isWrapperFunction && (arraySize > 0 || sArraySize > 0)) {
            _ss << _spaces << _baseTypeName << "* " << auxArrayName_ << ";\n";
        }

        if ((isWrapperFunction && zeroArrayDependents) ||
                (!isWrapperFunction && (arraySize > 0 || sArraySize > 0 || zeroArrayDependents))) {
            _ss << _spaces << U_INDEX_TYPE << " i;\n";
        }

        // loop indexes
        createIndexDeclaration();

        // clean-up
        std::string code = _ss.str();
        _ss.str("");

        return code;
    }

    inline void generateArrayContainersDeclaration(std::ostringstream& ss,
                                                   const std::vector<int>& atomicMaxForward,
                                                   const std::vector<int>& atomicMaxReverse) {
        int maxForward = -1;
        if (!atomicMaxForward.empty())
            maxForward = *std::max_element(atomicMaxForward.begin(), atomicMaxForward.end());

        int maxReverse = -1;
        if (!atomicMaxReverse.empty())
            maxReverse = *std::max_element(atomicMaxReverse.begin(), atomicMaxReverse.end());

        generateArrayContainersDeclaration(ss, maxForward, maxReverse);
    }

    virtual void generateArrayContainersDeclaration(std::ostringstream& ss,
                                                    int maxForwardOrder = -1,
                                                    int maxReverseOrder = -1) {
        if (maxForwardOrder >= 0 || maxReverseOrder >= 0) {
            ss << _spaces << "Array " << _ATOMIC_TX << "[" << (std::max(maxForwardOrder, maxReverseOrder) + 1) << "];\n";
            if (maxForwardOrder >= 0)
                ss << _spaces << "Array " << _ATOMIC_TY << ";\n";
            if (maxReverseOrder >= 0) {
                ss << _spaces << "Array " << _ATOMIC_PX << ";\n";
                ss << _spaces << "Array " << _ATOMIC_PY << "[" << (maxReverseOrder + 1) << "];\n";
            }
        }
    }

    virtual std::string generateDependentVariableDeclaration() {
        const std::vector<FuncArgument>& depArg = _nameGen->getDependent();
        CPPADCG_ASSERT_KNOWN(depArg.size() > 0,
                             "There must be at least one dependent argument");

        _ss << _spaces << "//dependent variables\n";
        for (size_t i = 0; i < depArg.size(); i++) {
            _ss << _spaces << argumentDeclaration(depArg[i]) << " = " << _outArgName << "[" << i << "];\n";
        }

        std::string code = _ss.str();
        _ss.str("");
        return code;
    }

    virtual std::string generateIndependentVariableDeclaration() {
        const std::vector<FuncArgument>& indArg = _nameGen->getIndependent();
        CPPADCG_ASSERT_KNOWN(indArg.size() > 0,
                             "There must be at least one independent argument");

        _ss << _spaces << "//independent variables\n";
        for (size_t i = 0; i < indArg.size(); i++) {
            _ss << _spaces << "const " << argumentDeclaration(indArg[i]) << " = " << _inArgName << "[" << i << "];\n";
        }

        std::string code = _ss.str();
        _ss.str("");
        return code;
    }

    inline std::string generateArgumentAtomicDcl() const {
        return "struct LangCAtomicFun " + _atomicArgName;
    }

    virtual std::string generateFunctionArgumentsDcl() const {
        std::string args = generateFunctionIndexArgumentsDcl();
        if (!args.empty())
            args += ", ";
        args += generateDefaultFunctionArgumentsDcl();

        return args;
    }

    virtual std::string generateDefaultFunctionArgumentsDcl() const {
        return _baseTypeName + " const *const * " + _inArgName +
                ", " + _baseTypeName + "*const * " + _outArgName +
                ", " + generateArgumentAtomicDcl();
    }

    virtual std::string generateFunctionIndexArgumentsDcl() const {
        std::string argtxt;
        for (size_t a = 0; a < _funcArgIndexes.size(); a++) {
            if (a > 0) argtxt += ", ";
            argtxt += U_INDEX_TYPE + " " + *_funcArgIndexes[a]->getName();
        }
        return argtxt;
    }

    virtual std::string generateDefaultFunctionArguments() const {
        return _inArgName + ", " + _outArgName + ", " + _atomicArgName;
    }

    virtual std::string generateFunctionIndexArguments() const {
        std::string argtxt;
        for (size_t a = 0; a < _funcArgIndexes.size(); a++) {
            if (a > 0) argtxt += ", ";
            argtxt += *_funcArgIndexes[a]->getName();
        }
        return argtxt;
    }

    inline void createIndexDeclaration();

    CPPAD_CG_C_LANG_FUNCNAME(abs)
    CPPAD_CG_C_LANG_FUNCNAME(acos)
    CPPAD_CG_C_LANG_FUNCNAME(asin)
    CPPAD_CG_C_LANG_FUNCNAME(atan)
    CPPAD_CG_C_LANG_FUNCNAME(cosh)
    CPPAD_CG_C_LANG_FUNCNAME(cos)
    CPPAD_CG_C_LANG_FUNCNAME(exp)
    CPPAD_CG_C_LANG_FUNCNAME(log)
    CPPAD_CG_C_LANG_FUNCNAME(sinh)
    CPPAD_CG_C_LANG_FUNCNAME(sin)
    CPPAD_CG_C_LANG_FUNCNAME(sqrt)
    CPPAD_CG_C_LANG_FUNCNAME(tanh)
    CPPAD_CG_C_LANG_FUNCNAME(tan)
    CPPAD_CG_C_LANG_FUNCNAME(pow)

#if CPPAD_USE_CPLUSPLUS_2011
    CPPAD_CG_C_LANG_FUNCNAME(erf)
    CPPAD_CG_C_LANG_FUNCNAME(asinh)
    CPPAD_CG_C_LANG_FUNCNAME(acosh)
    CPPAD_CG_C_LANG_FUNCNAME(atanh)
    CPPAD_CG_C_LANG_FUNCNAME(expm1)
    CPPAD_CG_C_LANG_FUNCNAME(log1p)
#endif

    inline virtual ~LanguageC() {
    }

    static inline void printIndexCondExpr(std::ostringstream& out,
                                          const std::vector<size_t>& info,
                                          const std::string& index) {
        CPPADCG_ASSERT_KNOWN(info.size() > 1 && info.size() % 2 == 0, "Invalid number of information elements for an index condition expression operation");

        size_t infoSize = info.size();
        for (size_t e = 0; e < infoSize; e += 2) {
            if (e > 0) {
                out << " || ";
            }
            size_t min = info[e];
            size_t max = info[e + 1];
            if (min == max) {
                out << index << " == " << min;
            } else if (min == 0) {
                out << index << " <= " << max;
            } else if (max == std::numeric_limits<size_t>::max()) {
                out << min << " <= " << index;
            } else {
                if (infoSize != 2)
                    out << "(";

                if (max - min == 1)
                    out << min << " == " << index << " || " << index << " == " << max;
                else
                    out << min << " <= " << index << " && " << index << " <= " << max;

                if (infoSize != 2)
                    out << ")";
            }
        }
    }

    /***********************************************************************
     * 
     **********************************************************************/

    static inline void printStaticIndexArray(std::ostringstream& os,
                                             const std::string& name,
                                             const std::vector<size_t>& values);

    static inline void printStaticIndexMatrix(std::ostringstream& os,
                                              const std::string& name,
                                              const std::map<size_t, std::map<size_t, size_t> >& values);

    /***********************************************************************
     * index patterns
     **********************************************************************/
    static inline void generateNames4RandomIndexPatterns(const std::set<RandomIndexPattern*>& randomPatterns);

    static inline void printRandomIndexPatternDeclaration(std::ostringstream& os,
                                                          const std::string& identation,
                                                          const std::set<RandomIndexPattern*>& randomPatterns);

    static inline std::string indexPattern2String(const IndexPattern& ip,
                                                  const Node& index);

    static inline std::string indexPattern2String(const IndexPattern& ip,
                                                  const std::string& index);

    static inline std::string indexPattern2String(const IndexPattern& ip,
                                                  const std::vector<const Node*>& indexes);

    static inline std::string indexPattern2String(const IndexPattern& ip,
                                                  const std::vector<const std::string*>& indexes);

    static inline std::string linearIndexPattern2String(const LinearIndexPattern& lip,
                                                        const Node& index);

    static inline std::string linearIndexPattern2String(const LinearIndexPattern& lip,
                                                        const std::string& index);

    static inline bool isOffsetBy(const IndexPattern* ip,
                                  const IndexPattern* refIp,
                                  long offset);

    static inline bool isOffsetBy(const LinearIndexPattern* lIp,
                                  const LinearIndexPattern* refLIp,
                                  long offset);

    static inline bool isOffsetBy(const LinearIndexPattern& lIp,
                                  const LinearIndexPattern& refLIp,
                                  long offset);


    static inline bool isOffsetBy(const SectionedIndexPattern* sIp,
                                  const SectionedIndexPattern* refSecp,
                                  long offset);

    static inline bool isOffsetBy(const SectionedIndexPattern& lIp,
                                  const SectionedIndexPattern& refSecp,
                                  long offset);

    static inline Plane2DIndexPattern* encapsulateIndexPattern(const LinearIndexPattern& refLIp,
                                                               size_t starti);

    static inline Plane2DIndexPattern* encapsulateIndexPattern(const SectionedIndexPattern& refSecp,
                                                               size_t starti);
protected:

    virtual void generateSourceCode(std::ostream& out,
                                    const std::unique_ptr<LanguageGenerationData<Base> >& info) override {

        const bool createFunction = !_functionName.empty();
        const bool multiFunction = createFunction && _maxAssigmentsPerFunction > 0 && _sources != nullptr;

        // clean up
        _code.str("");
        _ss.str("");
        _temporary.clear();
        _indentation = _spaces;
        funcArgDcl_ = "";
        localFuncArgDcl_ = "";
        localFuncArgs_ = "";
        auxArrayName_ = "";
        _currentLoops.clear();
        _atomicFuncArrays.clear();

        // save some info
        _info = info.get();
        _independentSize = info->independent.size();
        _dependent = &info->dependent;
        _nameGen = &info->nameGen;
        _minTemporaryVarID = info->minTemporaryVarID;
        const ArrayWrapper<CG<Base> >& dependent = info->dependent;
        const std::vector<Node*>& variableOrder = info->variableOrder;

        _tmpArrayValues.resize(_nameGen->getMaxTemporaryArrayVariableID());
        std::fill(_tmpArrayValues.begin(), _tmpArrayValues.end(), nullptr);
        _tmpSparseArrayValues.resize(_nameGen->getMaxTemporarySparseArrayVariableID());
        std::fill(_tmpSparseArrayValues.begin(), _tmpSparseArrayValues.end(), nullptr);

        /**
         * generate index array names (might be used for variable names)
         */
        generateNames4RandomIndexPatterns(info->indexRandomPatterns);

        /**
         * generate variable names
         */
        //generate names for the independent variables
        for (size_t j = 0; j < _independentSize; j++) {
            Node& op = *info->independent[j];
            if (op.getName() == nullptr) {
                op.setName(_nameGen->generateIndependent(op, getVariableID(op)));
            }
        }

        // generate names for the dependent variables (must be after naming independents)
        for (size_t i = 0; i < dependent.size(); i++) {
            Node* node = dependent[i].getOperationNode();
            if (node != nullptr && node->getOperationType() != CGOpCode::LoopEnd && node->getName() == nullptr) {
                if (node->getOperationType() == CGOpCode::LoopIndexedDep) {
                    size_t pos = node->getInfo()[0];
                    const IndexPattern* ip = info->loopDependentIndexPatterns[pos];
                    node->setName(_nameGen->generateIndexedDependent(*node, getVariableID(*node), *ip));

                } else {
                    node->setName(_nameGen->generateDependent(i));
                }
            }
        }

        /**
         * function variable declaration
         */
        const std::vector<FuncArgument>& indArg = _nameGen->getIndependent();
        const std::vector<FuncArgument>& depArg = _nameGen->getDependent();
        const std::vector<FuncArgument>& tmpArg = _nameGen->getTemporary();
        CPPADCG_ASSERT_KNOWN(indArg.size() > 0 && depArg.size() > 0,
                             "There must be at least one dependent and one independent argument");
        CPPADCG_ASSERT_KNOWN(tmpArg.size() == 3,
                             "There must be three temporary variables");

        if (createFunction) {
            funcArgDcl_ = generateFunctionArgumentsDcl();
            localFuncArgDcl_ = funcArgDcl_ + ", "
                    + argumentDeclaration(tmpArg[0]) + ", "
                    + argumentDeclaration(tmpArg[1]) + ", "
                    + argumentDeclaration(tmpArg[2]) + ", "
                    + U_INDEX_TYPE + "* " + _C_SPARSE_INDEX_ARRAY;
            localFuncArgs_ = generateDefaultFunctionArguments() + ", "
                    + tmpArg[0].name + ", "
                    + tmpArg[1].name + ", "
                    + tmpArg[2].name + ", "
                    + _C_SPARSE_INDEX_ARRAY;
        }

        auxArrayName_ = tmpArg[1].name + "p";

        /**
         * Determine the dependent variables that result from the same operations
         */
        // dependent variables indexes that are copies of other dependent variables
        std::set<size_t> dependentDuplicates;

        for (size_t i = 0; i < dependent.size(); i++) {
            Node* node = dependent[i].getOperationNode();
            if (node != nullptr) {
                CGOpCode type = node->getOperationType();
                if (type != CGOpCode::Inv && type != CGOpCode::LoopEnd) {
                    size_t varID = getVariableID(*node);
                    if (varID > 0) {
                        std::map<size_t, size_t>::const_iterator it2 = _dependentIDs.find(varID);
                        if (it2 == _dependentIDs.end()) {
                            _dependentIDs[getVariableID(*node)] = i;
                        } else {
                            // there can be several dependent variables with the same ID
                            dependentDuplicates.insert(i);
                        }
                    }
                }
            }
        }

        // the names of local functions
        std::vector<std::string> localFuncNames;
        if (multiFunction) {
            localFuncNames.reserve(variableOrder.size() / _maxAssigmentsPerFunction);
        }

        /**
         * non-constant variables
         */
        if (variableOrder.size() > 0) {
            // generate names for temporary variables
            for (Node* node : variableOrder) {
                CGOpCode op = node->getOperationType();
                if (!isDependent(*node) && op != CGOpCode::IndexDeclaration) {
                    // variable names for temporaries must always be created since they might have been used before with a different name/id
                    if (requiresVariableName(*node) && op != CGOpCode::ArrayCreation && op != CGOpCode::SparseArrayCreation) {
                        node->setName(_nameGen->generateTemporary(*node, getVariableID(*node)));
                    } else if (op == CGOpCode::ArrayCreation) {
                        node->setName(_nameGen->generateTemporaryArray(*node, getVariableID(*node)));
                    } else if (op == CGOpCode::SparseArrayCreation) {
                        node->setName(_nameGen->generateTemporarySparseArray(*node, getVariableID(*node)));
                    }
                }
            }

            /**
             * Source code generation magic!
             */
            if (info->zeroDependents) {
                // zero initial values
                const std::vector<FuncArgument>& depArg = _nameGen->getDependent();
                for (size_t i = 0; i < depArg.size(); i++) {
                    const FuncArgument& a = depArg[i];
                    if (a.array) {
                        _code << _indentation << "for(i = 0; i < " << _dependent->size() << "; i++) " << a.name << "[i]";
                    } else {
                        _code << _indentation << _nameGen->generateDependent(i);
                    }
                    _code << " = ";
                    printParameter(Base(0.0));
                    _code << ";\n";
                }
            }

            size_t assignCount = 0;
            for (size_t i = 0; i < variableOrder.size(); ++i) {
                Node* it = variableOrder[i];

                // check if a new function should start
                if (assignCount >= _maxAssigmentsPerFunction && multiFunction && _currentLoops.empty()) {
                    assignCount = 0;
                    saveLocalFunction(localFuncNames, localFuncNames.empty() && info->zeroDependents);
                }

                Node& node = *it;

                // a dependent variable assigned by a loop does require any source code (its done inside the loop)
                if (node.getOperationType() == CGOpCode::DependentRefRhs) {
                    continue; // nothing to do (this operation is right hand side only)
                } else if (node.getOperationType() == CGOpCode::TmpDcl) { // temporary variable declaration does not need any source code here
                    continue; // nothing to do (bogus operation)
                } else if (node.getOperationType() == CGOpCode::LoopIndexedDep) {
                    // try to detect a pattern and use a loop instead of individual assignments
                    i = printLoopIndexDeps(variableOrder, i);
                    continue;
                }

                assignCount += printAssigment(node);
            }

            if (localFuncNames.size() > 0 && assignCount > 0) {
                assignCount = 0;
                saveLocalFunction(localFuncNames, false);
            }
        }

        if (localFuncNames.size() > 0) {
            /**
             * Create the wrapper function which calls the other functions
             */
            CPPADCG_ASSERT_KNOWN(tmpArg[0].array,
                                 "The temporary variables must be saved in an array in order to generate multiple functions");

            _code << ATOMICFUN_STRUCT_DEFINITION << "\n\n";
            // forward declarations
            for (size_t i = 0; i < localFuncNames.size(); i++) {
                _code << "void " << localFuncNames[i] << "(" << localFuncArgDcl_ << ");\n";
            }
            _code << "\n"
                    << "void " << _functionName << "(" << funcArgDcl_ << ") {\n";
            _nameGen->customFunctionVariableDeclarations(_code);
            _code << generateIndependentVariableDeclaration() << "\n";
            _code << generateDependentVariableDeclaration() << "\n";
            _code << generateTemporaryVariableDeclaration(true, false,
                                                          info->atomicFunctionsMaxForward,
                                                          info->atomicFunctionsMaxReverse) << "\n";
            _nameGen->prepareCustomFunctionVariables(_code);
            for (size_t i = 0; i < localFuncNames.size(); i++) {
                _code << _spaces << localFuncNames[i] << "(" << localFuncArgs_ << ");\n";
            }
        }

        // dependent duplicates
        if (dependentDuplicates.size() > 0) {
            _code << _spaces << "// variable duplicates: " << dependentDuplicates.size() << "\n";
            for (size_t index : dependentDuplicates) {
                const CG<Base>& dep = (*_dependent)[index];
                std::string varName = _nameGen->generateDependent(index);
                const std::string& origVarName = *dep.getOperationNode()->getName();

                _code << _spaces << varName << " " << _depAssignOperation << " " << origVarName << ";\n";
            }
        }

        // constant dependent variables 
        bool commentWritten = false;
        for (size_t i = 0; i < dependent.size(); i++) {
            if (dependent[i].isParameter()) {
                if (!_ignoreZeroDepAssign || !dependent[i].isIdenticalZero()) {
                    if (!commentWritten) {
                        _code << _spaces << "// dependent variables without operations\n";
                        commentWritten = true;
                    }
                    std::string varName = _nameGen->generateDependent(i);
                    _code << _spaces << varName << " " << _depAssignOperation << " ";
                    printParameter(dependent[i].getValue());
                    _code << ";\n";
                }
            } else if (dependent[i].getOperationNode()->getOperationType() == CGOpCode::Inv) {
                if (!commentWritten) {
                    _code << _spaces << "// dependent variables without operations\n";
                    commentWritten = true;
                }
                std::string varName = _nameGen->generateDependent(i);
                const std::string& indepName = *dependent[i].getOperationNode()->getName();
                _code << _spaces << varName << " " << _depAssignOperation << " " << indepName << ";\n";
            }
        }

        /**
         * encapsulate the code in a function
         */
        if (createFunction) {
            if (localFuncNames.empty()) {
                _ss << "#include <math.h>\n"
                        "#include <stdio.h>\n\n"
                        << ATOMICFUN_STRUCT_DEFINITION << "\n\n"
                        << "void " << _functionName << "(" << funcArgDcl_ << ") {\n";
                _nameGen->customFunctionVariableDeclarations(_ss);
                _ss << generateIndependentVariableDeclaration() << "\n";
                _ss << generateDependentVariableDeclaration() << "\n";
                _ss << generateTemporaryVariableDeclaration(false, info->zeroDependents,
                                                            info->atomicFunctionsMaxForward,
                                                            info->atomicFunctionsMaxReverse) << "\n";
                _nameGen->prepareCustomFunctionVariables(_ss);
                _ss << _code.str();
                _nameGen->finalizeCustomFunctionVariables(_ss);
                _ss << "}\n\n";

                out << _ss.str();

                if (_sources != nullptr) {
                    (*_sources)[_functionName + ".c"] = _ss.str();
                }
            } else {
                _nameGen->finalizeCustomFunctionVariables(_code);
                _code << "}\n\n";

                (*_sources)[_functionName + ".c"] = _code.str();
            }
        } else {
            out << _code.str();
        }
    }

    inline size_t getVariableID(const Node& node) const {
        return _info->varId[node];
    }

    inline unsigned printAssigment(Node& node) {
        return printAssigment(node, node);
    }

    inline unsigned printAssigment(Node& nodeName,
                                   const Arg& nodeRhs) {
        if (nodeRhs.getOperation() != nullptr) {
            return printAssigment(nodeName, *nodeRhs.getOperation());
        } else {
            printAssigmentStart(nodeName);
            printParameter(*nodeRhs.getParameter());
            printAssigmentEnd(nodeName);
            return 1;
        }
    }

    inline unsigned printAssigment(Node& nodeName,
                                   Node& nodeRhs) {
        bool createsVar = directlyAssignsVariable(nodeRhs); // do we need to do the assignment here?
        if (!createsVar) {
            printAssigmentStart(nodeName);
        }
        unsigned lines = printExpressionNoVarCheck(nodeRhs);
        if (!createsVar) {
            printAssigmentEnd(nodeRhs);
        }

        if (nodeRhs.getOperationType() == CGOpCode::ArrayElement) {
            Node* array = nodeRhs.getArguments()[0].getOperation();
            size_t arrayId = getVariableID(*array);
            size_t pos = nodeRhs.getInfo()[0];
            if (array->getOperationType() == CGOpCode::ArrayCreation)
                _tmpArrayValues[arrayId - 1 + pos] = nullptr; // this could probably be removed!
            else
                _tmpSparseArrayValues[arrayId - 1 + pos] = nullptr; // this could probably be removed!
        }

        return lines;
    }

    inline virtual void printAssigmentStart(Node& op) {
        printAssigmentStart(op, createVariableName(op), isDependent(op));
    }

    inline virtual void printAssigmentStart(Node& node, const std::string& varName, bool isDep) {
        if (!isDep) {
            _temporary[getVariableID(node)] = &node;
        }

        _code << _indentation << varName << " ";
        if (isDep) {
            CGOpCode op = node.getOperationType();
            if (op == CGOpCode::DependentMultiAssign || (op == CGOpCode::LoopIndexedDep && node.getInfo()[1] == 1)) {
                _code << "+=";
            } else {
                _code << _depAssignOperation;
            }
        } else {
            _code << "=";
        }
        _code << " ";
    }

    inline virtual void printAssigmentEnd(Node& op) {
        _code << ";\n";
    }

    virtual std::string argumentDeclaration(const FuncArgument& funcArg) const {
        std::string dcl = _baseTypeName;
        if (funcArg.array) {
            dcl += "*";
        }
        return dcl + " " + funcArg.name;
    }

    virtual void saveLocalFunction(std::vector<std::string>& localFuncNames,
                                   bool zeroDependentArray) {
        _ss << _functionName << "__" << (localFuncNames.size() + 1);
        std::string funcName = _ss.str();
        _ss.str("");

        _ss << "#include <math.h>\n"
                "#include <stdio.h>\n\n"
                << ATOMICFUN_STRUCT_DEFINITION << "\n\n"
                << "void " << funcName << "(" << localFuncArgDcl_ << ") {\n";
        _nameGen->customFunctionVariableDeclarations(_ss);
        _ss << generateIndependentVariableDeclaration() << "\n";
        _ss << generateDependentVariableDeclaration() << "\n";
        size_t arraySize = _nameGen->getMaxTemporaryArrayVariableID();
        size_t sArraySize = _nameGen->getMaxTemporarySparseArrayVariableID();
        if (arraySize > 0 || sArraySize > 0) {
            _ss << _spaces << _baseTypeName << "* " << auxArrayName_ << ";\n";
        }

        generateArrayContainersDeclaration(_ss,
                                           _info->atomicFunctionsMaxForward,
                                           _info->atomicFunctionsMaxReverse);

        if (arraySize > 0 || sArraySize > 0 || zeroDependentArray) {
            _ss << _spaces << U_INDEX_TYPE << " i;\n";
        }

        // loop indexes
        createIndexDeclaration();

        _nameGen->prepareCustomFunctionVariables(_ss);
        _ss << _code.str();
        _nameGen->finalizeCustomFunctionVariables(_ss);
        _ss << "}\n\n";

        (*_sources)[funcName + ".c"] = _ss.str();
        localFuncNames.push_back(funcName);

        _code.str("");
        _ss.str("");
    }

    virtual bool createsNewVariable(const Node& var,
                                    size_t totalUseCount) const override {
        CGOpCode op = var.getOperationType();
        if (totalUseCount > 1) {
            return op != CGOpCode::ArrayElement && op != CGOpCode::Index && op != CGOpCode::IndexDeclaration && op != CGOpCode::Tmp;
        } else {
            return ( op == CGOpCode::ArrayCreation ||
                    op == CGOpCode::SparseArrayCreation ||
                    op == CGOpCode::AtomicForward ||
                    op == CGOpCode::AtomicReverse ||
                    op == CGOpCode::ComLt ||
                    op == CGOpCode::ComLe ||
                    op == CGOpCode::ComEq ||
                    op == CGOpCode::ComGe ||
                    op == CGOpCode::ComGt ||
                    op == CGOpCode::ComNe ||
                    op == CGOpCode::LoopIndexedDep ||
                    op == CGOpCode::LoopIndexedTmp ||
                    op == CGOpCode::IndexAssign ||
                    op == CGOpCode::Assign) &&
                    op != CGOpCode::CondResult;
        }
    }

    virtual bool requiresVariableName(const Node& var) const {
        CGOpCode op = var.getOperationType();
        if (_info->totalUseCount.get(var) > 1) {
            return (op != CGOpCode::Pri &&
                    op != CGOpCode::AtomicForward &&
                    op != CGOpCode::AtomicReverse &&
                    op != CGOpCode::LoopStart &&
                    op != CGOpCode::LoopEnd &&
                    op != CGOpCode::Index &&
                    op != CGOpCode::IndexAssign &&
                    op != CGOpCode::StartIf &&
                    op != CGOpCode::ElseIf &&
                    op != CGOpCode::Else &&
                    op != CGOpCode::EndIf &&
                    op != CGOpCode::CondResult &&
                    op != CGOpCode::LoopIndexedTmp &&
                    op != CGOpCode::Tmp);
        } else {
            return isCondAssign(op);
        }
    }

    /**
     * Whether or not this operation assign its expression to a variable by
     * itself.
     * 
     * @param var the operation node
     * @return 
     */
    virtual bool directlyAssignsVariable(const Node& var) const {
        CGOpCode op = var.getOperationType();
        return isCondAssign(op) ||
                op == CGOpCode::Pri ||
                op == CGOpCode::ArrayCreation ||
                op == CGOpCode::SparseArrayCreation ||
                op == CGOpCode::AtomicForward ||
                op == CGOpCode::AtomicReverse ||
                op == CGOpCode::DependentMultiAssign ||
                op == CGOpCode::LoopStart ||
                op == CGOpCode::LoopEnd ||
                op == CGOpCode::IndexAssign ||
                op == CGOpCode::StartIf ||
                op == CGOpCode::ElseIf ||
                op == CGOpCode::Else ||
                op == CGOpCode::EndIf ||
                op == CGOpCode::CondResult ||
                op == CGOpCode::IndexDeclaration;
    }

    virtual bool requiresVariableArgument(enum CGOpCode op, size_t argIndex) const override {
        return op == CGOpCode::Sign || op == CGOpCode::CondResult || op == CGOpCode::Pri;
    }

    inline const std::string& createVariableName(Node& var) {
        CGOpCode op = var.getOperationType();
        CPPADCG_ASSERT_UNKNOWN(getVariableID(var) > 0);
        CPPADCG_ASSERT_UNKNOWN(op != CGOpCode::AtomicForward);
        CPPADCG_ASSERT_UNKNOWN(op != CGOpCode::AtomicReverse);
        CPPADCG_ASSERT_UNKNOWN(op != CGOpCode::LoopStart);
        CPPADCG_ASSERT_UNKNOWN(op != CGOpCode::LoopEnd);
        CPPADCG_ASSERT_UNKNOWN(op != CGOpCode::Index);
        CPPADCG_ASSERT_UNKNOWN(op != CGOpCode::IndexAssign);
        CPPADCG_ASSERT_UNKNOWN(op != CGOpCode::IndexDeclaration);

        if (var.getName() == nullptr) {
            if (op == CGOpCode::ArrayCreation) {
                var.setName(_nameGen->generateTemporaryArray(var, getVariableID(var)));

            } else if (op == CGOpCode::SparseArrayCreation) {
                var.setName(_nameGen->generateTemporarySparseArray(var, getVariableID(var)));

            } else if (op == CGOpCode::LoopIndexedDep) {
                size_t pos = var.getInfo()[0];
                const IndexPattern* ip = _info->loopDependentIndexPatterns[pos];
                var.setName(_nameGen->generateIndexedDependent(var, getVariableID(var), *ip));

            } else if (op == CGOpCode::LoopIndexedIndep) {
                size_t pos = var.getInfo()[1];
                const IndexPattern* ip = _info->loopIndependentIndexPatterns[pos];
                var.setName(_nameGen->generateIndexedIndependent(var, getVariableID(var), *ip));

            } else if (getVariableID(var) <= _independentSize) {
                // independent variable
                var.setName(_nameGen->generateIndependent(var, getVariableID(var)));

            } else if (getVariableID(var) < _minTemporaryVarID) {
                // dependent variable
                std::map<size_t, size_t>::const_iterator it = _dependentIDs.find(getVariableID(var));
                CPPADCG_ASSERT_UNKNOWN(it != _dependentIDs.end());

                size_t index = it->second;
                var.setName(_nameGen->generateDependent(index));
            } else if (op == CGOpCode::Pri) {
                CPPADCG_ASSERT_KNOWN(var.getArguments().size() == 1, "Invalid number of arguments for print operation");
                Node* tmpVar = var.getArguments()[0].getOperation();
                CPPADCG_ASSERT_KNOWN(tmpVar != nullptr, "Invalid argument for print operation");
                return createVariableName(*tmpVar);

            } else if (op == CGOpCode::LoopIndexedTmp || op == CGOpCode::Tmp) {
                CPPADCG_ASSERT_KNOWN(var.getArguments().size() >= 1, "Invalid number of arguments for loop indexed temporary operation");
                Node* tmpVar = var.getArguments()[0].getOperation();
                CPPADCG_ASSERT_KNOWN(tmpVar != nullptr && tmpVar->getOperationType() == CGOpCode::TmpDcl, "Invalid arguments for loop indexed temporary operation");
                return createVariableName(*tmpVar);

            } else {
                // temporary variable
                var.setName(_nameGen->generateTemporary(var, getVariableID(var)));
            }
        }


        return *var.getName();
    }

    virtual bool requiresVariableDependencies() const override {
        return false;
    }

    virtual void printIndependentVariableName(Node& op) {
        CPPADCG_ASSERT_KNOWN(op.getArguments().size() == 0, "Invalid number of arguments for independent variable");

        _code << _nameGen->generateIndependent(op, getVariableID(op));
    }

    virtual unsigned print(const Arg& arg) {
        if (arg.getOperation() != nullptr) {
            // expression
            return printExpression(*arg.getOperation());
        } else {
            // parameter
            printParameter(*arg.getParameter());
            return 1;
        }
    }

    virtual unsigned printExpression(Node& op) {
        if (getVariableID(op) > 0) {
            // use variable name
            _code << createVariableName(op);
            return 1;
        } else {
            // print expression code
            return printExpressionNoVarCheck(op);
        }
    }

    virtual unsigned printExpressionNoVarCheck(Node& node) {
        CGOpCode op = node.getOperationType();
        switch (op) {
            case CGOpCode::ArrayCreation:
                printArrayCreationOp(node);
                break;
            case CGOpCode::SparseArrayCreation:
                printSparseArrayCreationOp(node);
                break;
            case CGOpCode::ArrayElement:
                printArrayElementOp(node);
                break;
            case CGOpCode::Assign:
                return printAssignOp(node);

            case CGOpCode::Abs:
            case CGOpCode::Acos:
            case CGOpCode::Asin:
            case CGOpCode::Atan:
            case CGOpCode::Cosh:
            case CGOpCode::Cos:
            case CGOpCode::Exp:
            case CGOpCode::Log:
            case CGOpCode::Sinh:
            case CGOpCode::Sin:
            case CGOpCode::Sqrt:
            case CGOpCode::Tanh:
            case CGOpCode::Tan:
#if CPPAD_USE_CPLUSPLUS_2011
            case CGOpCode::Erf:
            case CGOpCode::Asinh:
            case CGOpCode::Acosh:
            case CGOpCode::Atanh:
            case CGOpCode::Expm1:
            case CGOpCode::Log1p:
#endif
                printUnaryFunction(node);
                break;
            case CGOpCode::AtomicForward: // atomicFunction.forward(q, p, vx, vy, tx, ty)
                printAtomicForwardOp(node);
                break;
            case CGOpCode::AtomicReverse: // atomicFunction.reverse(p, tx, ty, px, py)
                printAtomicReverseOp(node);
                break;
            case CGOpCode::Add:
                printOperationAdd(node);
                break;
            case CGOpCode::Alias:
                return printOperationAlias(node);

            case CGOpCode::ComLt:
            case CGOpCode::ComLe:
            case CGOpCode::ComEq:
            case CGOpCode::ComGe:
            case CGOpCode::ComGt:
            case CGOpCode::ComNe:
                printConditionalAssignment(node);
                break;
            case CGOpCode::Div:
                printOperationDiv(node);
                break;
            case CGOpCode::Inv:
                printIndependentVariableName(node);
                break;
            case CGOpCode::Mul:
                printOperationMul(node);
                break;
            case CGOpCode::Pow:
                printPowFunction(node);
                break;
            case CGOpCode::Pri:
                printPrintOperation(node);
                break;
            case CGOpCode::Sign:
                printSignFunction(node);
                break;
            case CGOpCode::Sub:
                printOperationMinus(node);
                break;

            case CGOpCode::UnMinus:
                printOperationUnaryMinus(node);
                break;

            case CGOpCode::DependentMultiAssign:
                return printDependentMultiAssign(node);

            case CGOpCode::Index:
                return 0; // nothing to do
            case CGOpCode::IndexAssign:
                printIndexAssign(node);
                break;
            case CGOpCode::IndexDeclaration:
                return 0; // already done

            case CGOpCode::LoopStart:
                printLoopStart(node);
                break;
            case CGOpCode::LoopIndexedIndep:
                printLoopIndexedIndep(node);
                break;
            case CGOpCode::LoopIndexedDep:
                printLoopIndexedDep(node);
                break;
            case CGOpCode::LoopIndexedTmp:
                printLoopIndexedTmp(node);
                break;
            case CGOpCode::TmpDcl:
                // nothing to do
                return 0;
            case CGOpCode::Tmp:
                printTmpVar(node);
                break;
            case CGOpCode::LoopEnd:
                printLoopEnd(node);
                break;
            case CGOpCode::IndexCondExpr:
                printIndexCondExprOp(node);
                break;
            case CGOpCode::StartIf:
                printStartIf(node);
                break;
            case CGOpCode::ElseIf:
                printElseIf(node);
                break;
            case CGOpCode::Else:
                printElse(node);
                break;
            case CGOpCode::EndIf:
                printEndIf(node);
                break;
            case CGOpCode::CondResult:
                printCondResult(node);
                break;
            case CGOpCode::UserCustom:
                printUserCustom(node);
                break;
            default:
                throw CGException("Unknown operation code '", op, "'.");
        }
        return 1;
    }

    virtual unsigned printAssignOp(Node& node) {
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() == 1, "Invalid number of arguments for assign operation");

        return print(node.getArguments()[0]);
    }

    virtual void printUnaryFunction(Node& op) {
        CPPADCG_ASSERT_KNOWN(op.getArguments().size() == 1, "Invalid number of arguments for unary function");

        switch (op.getOperationType()) {
            case CGOpCode::Abs:
                _code << absFuncName();
                break;
            case CGOpCode::Acos:
                _code << acosFuncName();
                break;
            case CGOpCode::Asin:
                _code << asinFuncName();
                break;
            case CGOpCode::Atan:
                _code << atanFuncName();
                break;
            case CGOpCode::Cosh:
                _code << coshFuncName();
                break;
            case CGOpCode::Cos:
                _code << cosFuncName();
                break;
            case CGOpCode::Exp:
                _code << expFuncName();
                break;
            case CGOpCode::Log:
                _code << logFuncName();
                break;
            case CGOpCode::Sinh:
                _code << sinhFuncName();
                break;
            case CGOpCode::Sin:
                _code << sinFuncName();
                break;
            case CGOpCode::Sqrt:
                _code << sqrtFuncName();
                break;
            case CGOpCode::Tanh:
                _code << tanhFuncName();
                break;
            case CGOpCode::Tan:
                _code << tanFuncName();
                break;
#if CPPAD_USE_CPLUSPLUS_2011
            case CGOpCode::Erf:
                _code << erfFuncName();
                break;
            case CGOpCode::Asinh:
                _code << asinhFuncName();
                break;
            case CGOpCode::Acosh:
                _code << acoshFuncName();
                break;
            case CGOpCode::Atanh:
                _code << atanhFuncName();
                break;
            case CGOpCode::Expm1:
                _code << expm1FuncName();
                break;
            case CGOpCode::Log1p:
                _code << log1pFuncName();
                break;
#endif
            default:
                throw CGException("Unknown function name for operation code '", op.getOperationType(), "'.");
        }

        _code << "(";
        print(op.getArguments()[0]);
        _code << ")";
    }

    virtual void printPowFunction(Node& op) {
        CPPADCG_ASSERT_KNOWN(op.getArguments().size() == 2, "Invalid number of arguments for pow() function");

        _code << powFuncName() << "(";
        print(op.getArguments()[0]);
        _code << ", ";
        print(op.getArguments()[1]);
        _code << ")";
    }

    virtual void printSignFunction(Node& op) {
        CPPADCG_ASSERT_KNOWN(op.getArguments().size() == 1, "Invalid number of arguments for sign() function");
        CPPADCG_ASSERT_UNKNOWN(op.getArguments()[0].getOperation() != nullptr);
        CPPADCG_ASSERT_UNKNOWN(getVariableID(*op.getArguments()[0].getOperation()) > 0);

        Node& arg = *op.getArguments()[0].getOperation();

        const std::string& argName = createVariableName(arg);

        _code << "(" << argName << " " << _C_COMP_OP_GT << " ";
        printParameter(Base(0.0));
        _code << "?";
        printParameter(Base(1.0));
        _code << ":(" << argName << " " << _C_COMP_OP_LT << " ";
        printParameter(Base(0.0));
        _code << "?";
        printParameter(Base(-1.0));
        _code << ":";
        printParameter(Base(0.0));
        _code << "))";
    }

    virtual unsigned printOperationAlias(Node& op) {
        CPPADCG_ASSERT_KNOWN(op.getArguments().size() == 1, "Invalid number of arguments for alias");
        return print(op.getArguments()[0]);
    }

    virtual void printOperationAdd(Node& op) {
        CPPADCG_ASSERT_KNOWN(op.getArguments().size() == 2, "Invalid number of arguments for addition");

        const Arg& left = op.getArguments()[0];
        const Arg& right = op.getArguments()[1];

        if(right.getParameter() == nullptr || (*right.getParameter() >= 0)) {
            print(left);
            _code << " + ";
            print(right);
        } else {
            // right has a negative parameter so we would get v0 + -v1
            print(left);
            _code << " - ";
            printParameter(-*right.getParameter()); // make it positive
        }
    }

    virtual void printOperationMinus(Node& op) {
        CPPADCG_ASSERT_KNOWN(op.getArguments().size() == 2, "Invalid number of arguments for subtraction");

        const Arg& left = op.getArguments()[0];
        const Arg& right = op.getArguments()[1];

        if(right.getParameter() == nullptr || (*right.getParameter() >= 0)) {
            bool encloseRight = encloseInParenthesesMul(right.getOperation());

            print(left);
            _code << " - ";
            if (encloseRight) {
                _code << "(";
            }
            print(right);
            if (encloseRight) {
                _code << ")";
            }
        } else {
            // right has a negative parameter so we would get v0 - -v1
            print(left);
            _code << " + ";
            printParameter(-*right.getParameter()); // make it positive
        }
    }

    inline bool encloseInParenthesesDiv(const Node* node) const {
        while (node != nullptr) {
            if (getVariableID(*node) != 0)
                return false;
            if (node->getOperationType() == CGOpCode::Alias)
                node = node->getArguments()[0].getOperation();
            else
                break;
        }
        return node != nullptr &&
                getVariableID(*node) == 0 &&
                !isFunction(node->getOperationType());
    }

    virtual void printOperationDiv(Node& op) {
        CPPADCG_ASSERT_KNOWN(op.getArguments().size() == 2, "Invalid number of arguments for division");

        const Arg& left = op.getArguments()[0];
        const Arg& right = op.getArguments()[1];

        bool encloseLeft = encloseInParenthesesDiv(left.getOperation());
        bool encloseRight = encloseInParenthesesDiv(right.getOperation());

        if (encloseLeft) {
            _code << "(";
        }
        print(left);
        if (encloseLeft) {
            _code << ")";
        }
        _code << " / ";
        if (encloseRight) {
            _code << "(";
        }
        print(right);
        if (encloseRight) {
            _code << ")";
        }
    }

    inline bool encloseInParenthesesMul(const Node* node) const {
        while (node != nullptr) {
            if (getVariableID(*node) != 0)
                return false;
            else if (node->getOperationType() == CGOpCode::Alias)
                node = node->getArguments()[0].getOperation();
            else
                break;
        }
        return node != nullptr &&
                getVariableID(*node) == 0 &&
                node->getOperationType() != CGOpCode::Div &&
                node->getOperationType() != CGOpCode::Mul &&
                !isFunction(node->getOperationType());
    }

    virtual void printOperationMul(Node& op) {
        CPPADCG_ASSERT_KNOWN(op.getArguments().size() == 2, "Invalid number of arguments for multiplication");

        const Arg& left = op.getArguments()[0];
        const Arg& right = op.getArguments()[1];

        bool encloseLeft = encloseInParenthesesMul(left.getOperation());
        bool encloseRight = encloseInParenthesesMul(right.getOperation());

        if (encloseLeft) {
            _code << "(";
        }
        print(left);
        if (encloseLeft) {
            _code << ")";
        }
        _code << " * ";
        if (encloseRight) {
            _code << "(";
        }
        print(right);
        if (encloseRight) {
            _code << ")";
        }
    }

    virtual void printOperationUnaryMinus(Node& op) {
        CPPADCG_ASSERT_KNOWN(op.getArguments().size() == 1, "Invalid number of arguments for unary minus");

        const Arg& arg = op.getArguments()[0];

        bool enclose = encloseInParenthesesMul(arg.getOperation());

        _code << "-";
        if (enclose) {
            _code << "(";
        } else {
            _code << " "; // there may be several - together -> space required
        }
        print(arg);
        if (enclose) {
            _code << ")";
        }
    }

    virtual void printPrintOperation(const Node& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::Pri, "Invalid node type");
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() >= 1, "Invalid number of arguments for print operation");

        const PrintOperationNode<Base>& pnode = static_cast<const PrintOperationNode<Base>&> (node);
        std::string before = pnode.getBeforeString();
        replaceString(before, "\n", "\\n");
        replaceString(before, "\"", "\\\"");
        std::string after = pnode.getAfterString();
        replaceString(after, "\n", "\\n");
        replaceString(after, "\"", "\\\"");

        _code << _indentation << "fprintf(stderr, \"" << before << getPrintfBaseFormat() << after << "\"";
        const std::vector<Arg>& args = pnode.getArguments();
        for (size_t a = 0; a < args.size(); a++) {
            _code << ", ";
            print(args[a]);
        }
        _code << ");\n";
    }

    virtual void printConditionalAssignment(Node& node) {
        CPPADCG_ASSERT_UNKNOWN(getVariableID(node) > 0);

        const std::vector<Arg>& args = node.getArguments();
        const Arg &left = args[0];
        const Arg &right = args[1];
        const Arg &trueCase = args[2];
        const Arg &falseCase = args[3];

        bool isDep = isDependent(node);
        const std::string& varName = createVariableName(node);

        if ((trueCase.getParameter() != nullptr && falseCase.getParameter() != nullptr && *trueCase.getParameter() == *falseCase.getParameter()) ||
                (trueCase.getOperation() != nullptr && falseCase.getOperation() != nullptr && trueCase.getOperation() == falseCase.getOperation())) {
            // true and false cases are the same
            printAssigmentStart(node, varName, isDep);
            print(trueCase);
            printAssigmentEnd(node);
        } else {
            _code << _indentation << "if( ";
            print(left);
            _code << " " << getComparison(node.getOperationType()) << " ";
            print(right);
            _code << " ) {\n";
            _code << _spaces;
            printAssigmentStart(node, varName, isDep);
            print(trueCase);
            printAssigmentEnd(node);
            _code << _indentation << "} else {\n";
            _code << _spaces;
            printAssigmentStart(node, varName, isDep);
            print(falseCase);
            printAssigmentEnd(node);
            _code << _indentation << "}\n";
        }
    }

    inline bool isSameArgument(const Arg& newArg,
                               const Arg* oldArg) {
        if (oldArg != nullptr) {
            if (oldArg->getParameter() != nullptr) {
                if (newArg.getParameter() != nullptr) {
                    return (*newArg.getParameter() == *oldArg->getParameter());
                }
            } else {
                return (newArg.getOperation() == oldArg->getOperation());
            }
        }
        return false;
    }

    virtual void printArrayCreationOp(Node& op);

    virtual void printSparseArrayCreationOp(Node& op);

    inline void printArrayStructInit(const std::string& dataArrayName,
                                     size_t pos,
                                     const std::vector<Node*>& arrays,
                                     size_t k);

    inline void printArrayStructInit(const std::string& dataArrayName,
                                     Node& array);

    inline void markArrayChanged(Node& ty);

    inline size_t printArrayCreationUsingLoop(size_t startPos,
                                              Node& array,
                                              size_t startj,
                                              std::vector<const Arg*>& tmpArrayValues);

    inline std::string getTempArrayName(const Node& op);

    virtual void printArrayElementOp(Node& op);

    virtual void printAtomicForwardOp(Node& atomicFor) {
        CPPADCG_ASSERT_KNOWN(atomicFor.getInfo().size() == 3, "Invalid number of information elements for atomic forward operation");
        int q = atomicFor.getInfo()[1];
        int p = atomicFor.getInfo()[2];
        size_t p1 = p + 1;
        const std::vector<Arg>& opArgs = atomicFor.getArguments();
        CPPADCG_ASSERT_KNOWN(opArgs.size() == p1 * 2, "Invalid number of arguments for atomic forward operation");

        size_t id = atomicFor.getInfo()[0];
        size_t atomicIndex = _info->atomicFunctionId2Index.at(id);

        std::vector<Node*> tx(p1), ty(p1);
        for (size_t k = 0; k < p1; k++) {
            tx[k] = opArgs[0 * p1 + k].getOperation();
            ty[k] = opArgs[1 * p1 + k].getOperation();
        }

        CPPADCG_ASSERT_KNOWN(tx[0]->getOperationType() == CGOpCode::ArrayCreation, "Invalid array type");
        CPPADCG_ASSERT_KNOWN(p == 0 || tx[1]->getOperationType() == CGOpCode::SparseArrayCreation, "Invalid array type");

        CPPADCG_ASSERT_KNOWN(ty[p]->getOperationType() == CGOpCode::ArrayCreation, "Invalid array type");

        // tx
        for (size_t k = 0; k < p1; k++) {
            printArrayStructInit(_ATOMIC_TX, k, tx, k); // also does indentation
        }
        // ty
        printArrayStructInit(_ATOMIC_TY, *ty[p]); // also does indentation
        _ss.str("");

        _code << _indentation << "atomicFun.forward(atomicFun.libModel, "
                << atomicIndex << ", " << q << ", " << p << ", "
                << _ATOMIC_TX << ", &" << _ATOMIC_TY << "); // "
                << _info->atomicFunctionId2Name.at(id)
                << "\n";

        /**
         * the values of ty are now changed
         */
        markArrayChanged(*ty[p]);
    }

    virtual void printAtomicReverseOp(Node& atomicRev) {
        CPPADCG_ASSERT_KNOWN(atomicRev.getInfo().size() == 2, "Invalid number of information elements for atomic reverse operation");
        int p = atomicRev.getInfo()[1];
        size_t p1 = p + 1;
        const std::vector<Arg>& opArgs = atomicRev.getArguments();
        CPPADCG_ASSERT_KNOWN(opArgs.size() == p1 * 4, "Invalid number of arguments for atomic reverse operation");

        size_t id = atomicRev.getInfo()[0];
        size_t atomicIndex = _info->atomicFunctionId2Index.at(id);
        std::vector<Node*> tx(p1), px(p1), py(p1);
        for (size_t k = 0; k < p1; k++) {
            tx[k] = opArgs[0 * p1 + k].getOperation();
            px[k] = opArgs[2 * p1 + k].getOperation();
            py[k] = opArgs[3 * p1 + k].getOperation();
        }

        CPPADCG_ASSERT_KNOWN(tx[0]->getOperationType() == CGOpCode::ArrayCreation, "Invalid array type");
        CPPADCG_ASSERT_KNOWN(p == 0 || tx[1]->getOperationType() == CGOpCode::SparseArrayCreation, "Invalid array type");

        CPPADCG_ASSERT_KNOWN(px[0]->getOperationType() == CGOpCode::ArrayCreation, "Invalid array type");

        CPPADCG_ASSERT_KNOWN(py[0]->getOperationType() == CGOpCode::SparseArrayCreation, "Invalid array type");
        CPPADCG_ASSERT_KNOWN(p == 0 || py[1]->getOperationType() == CGOpCode::ArrayCreation, "Invalid array type");

        // tx
        for (size_t k = 0; k < p1; k++) {
            printArrayStructInit(_ATOMIC_TX, k, tx, k); // also does indentation
        }
        // py
        for (size_t k = 0; k < p1; k++) {
            printArrayStructInit(_ATOMIC_PY, k, py, k); // also does indentation
        }
        // px
        printArrayStructInit(_ATOMIC_PX, *px[0]); // also does indentation
        _ss.str("");

        _code << _indentation << "atomicFun.reverse(atomicFun.libModel, "
                << atomicIndex << ", " << p << ", "
                << _ATOMIC_TX << ", &" << _ATOMIC_PX << ", " << _ATOMIC_PY << "); // "
                << _info->atomicFunctionId2Name.at(id)
                << "\n";

        /**
         * the values of px are now changed
         */
        markArrayChanged(*px[0]);
    }

    virtual unsigned printDependentMultiAssign(Node& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::DependentMultiAssign, "Invalid node type");
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() > 0, "Invalid number of arguments");

        const std::vector<Arg>& args = node.getArguments();
        for (size_t a = 0; a < args.size(); a++) {
            bool useArg = false;
            const Arg& arg = args[a];
            if (arg.getParameter() != nullptr) {
                useArg = true;
            } else {
                CGOpCode op = arg.getOperation()->getOperationType();
                useArg = op != CGOpCode::DependentRefRhs && op != CGOpCode::LoopEnd && op != CGOpCode::EndIf;
            }

            if (useArg) {
                printAssigment(node, arg); // ignore other arguments!
                return 1;
            }
        }
        return 0;
    }

    virtual void printLoopStart(Node& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::LoopStart, "Invalid node type");

        LoopStartOperationNode<Base>& lnode = static_cast<LoopStartOperationNode<Base>&> (node);
        _currentLoops.push_back(&lnode);

        const std::string& jj = *lnode.getIndex().getName();
        std::string iterationCount;
        if (lnode.getIterationCountNode() != nullptr) {
            iterationCount = *lnode.getIterationCountNode()->getIndex().getName();
        } else {
            std::ostringstream oss;
            oss << lnode.getIterationCount();
            iterationCount = oss.str();
        }

        _code << _spaces << "for("
                << jj << " = 0; "
                << jj << " < " << iterationCount << "; "
                << jj << "++) {\n";
        _indentation += _spaces;
    }

    virtual void printLoopEnd(Node& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::LoopEnd, "Invalid node type");

        _indentation.resize(_indentation.size() - _spaces.size());

        _code << _indentation << "}\n";

        _currentLoops.pop_back();
    }


    virtual size_t printLoopIndexDeps(const std::vector<Node*>& variableOrder,
                                      size_t pos);

    virtual size_t printLoopIndexedDepsUsingLoop(const std::vector<Node*>& variableOrder,
                                                 size_t starti);

    virtual void printLoopIndexedDep(Node& node);

    virtual void printLoopIndexedIndep(Node& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::LoopIndexedIndep, "Invalid node type");
        CPPADCG_ASSERT_KNOWN(node.getInfo().size() == 1, "Invalid number of information elements for loop indexed independent operation");

        // CGLoopIndexedIndepOp
        size_t pos = node.getInfo()[1];
        const IndexPattern* ip = _info->loopIndependentIndexPatterns[pos];
        _code << _nameGen->generateIndexedIndependent(node, getVariableID(node), *ip);
    }

    virtual void printLoopIndexedTmp(Node& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::LoopIndexedTmp, "Invalid node type");
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() == 2, "Invalid number of arguments for loop indexed temporary operation");
        Node* tmpVar = node.getArguments()[0].getOperation();
        CPPADCG_ASSERT_KNOWN(tmpVar != nullptr && tmpVar->getOperationType() == CGOpCode::TmpDcl, "Invalid arguments for loop indexed temporary operation");

        print(node.getArguments()[1]);
    }

    virtual void printTmpVar(Node& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::Tmp, "Invalid node type");
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() > 0, "Invalid number of arguments for temporary variable usage operation");
        Node* tmpVar = node.getArguments()[0].getOperation();
        CPPADCG_ASSERT_KNOWN(tmpVar != nullptr && tmpVar->getOperationType() == CGOpCode::TmpDcl, "Invalid arguments for loop indexed temporary operation");

        _code << *tmpVar->getName();
    }

    virtual void printIndexAssign(Node& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::IndexAssign, "Invalid node type");
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() > 0, "Invalid number of arguments for an index assignment operation");

        IndexAssignOperationNode<Base>& inode = static_cast<IndexAssignOperationNode<Base>&> (node);

        const IndexPattern& ip = inode.getIndexPattern();
        _code << _indentation << (*inode.getIndex().getName())
                << " = " << indexPattern2String(ip, inode.getIndexPatternIndexes()) << ";\n";
    }

    virtual void printIndexCondExprOp(Node& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::IndexCondExpr, "Invalid node type");
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() == 1, "Invalid number of arguments for an index condition expression operation");
        CPPADCG_ASSERT_KNOWN(node.getArguments()[0].getOperation() != nullptr, "Invalid argument for an index condition expression operation");
        CPPADCG_ASSERT_KNOWN(node.getArguments()[0].getOperation()->getOperationType() == CGOpCode::Index, "Invalid argument for an index condition expression operation");

        const std::vector<size_t>& info = node.getInfo();

        IndexOperationNode<Base>& iterationIndexOp = static_cast<IndexOperationNode<Base>&> (*node.getArguments()[0].getOperation());
        const std::string& index = *iterationIndexOp.getIndex().getName();

        printIndexCondExpr(_code, info, index);
    }

    virtual void printStartIf(Node& node) {
        /**
         * the first argument is the condition, following arguments are
         * just extra dependencies that must be defined outside the if
         */
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::StartIf, "Invalid node type");
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() >= 1, "Invalid number of arguments for an 'if start' operation");
        CPPADCG_ASSERT_KNOWN(node.getArguments()[0].getOperation() != nullptr, "Invalid argument for an 'if start' operation");

        _code << _indentation << "if(";
        printIndexCondExprOp(*node.getArguments()[0].getOperation());
        _code << ") {\n";

        _indentation += _spaces;
    }

    virtual void printElseIf(Node& node) {
        /**
         * the first argument is the condition, the second argument is the 
         * if start node, the following arguments are assignments in the
         * previous if branch
         */
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::ElseIf, "Invalid node type");
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() >= 2, "Invalid number of arguments for an 'else if' operation");
        CPPADCG_ASSERT_KNOWN(node.getArguments()[0].getOperation() != nullptr, "Invalid argument for an 'else if' operation");
        CPPADCG_ASSERT_KNOWN(node.getArguments()[1].getOperation() != nullptr, "Invalid argument for an 'else if' operation");

        _indentation.resize(_indentation.size() - _spaces.size());

        _code << _indentation << "} else if(";
        printIndexCondExprOp(*node.getArguments()[1].getOperation());
        _code << ") {\n";

        _indentation += _spaces;
    }

    virtual void printElse(Node& node) {
        /**
         * the first argument is the  if start node, the following arguments
         * are assignments in the previous if branch
         */
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::Else, "Invalid node type");
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() >= 1, "Invalid number of arguments for an 'else' operation");

        _indentation.resize(_indentation.size() - _spaces.size());

        _code << _indentation << "} else {\n";

        _indentation += _spaces;
    }

    virtual void printEndIf(Node& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::EndIf, "Invalid node type for an 'end if' operation");

        _indentation.resize(_indentation.size() - _spaces.size());

        _code << _indentation << "}\n";
    }

    virtual void printCondResult(Node& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::CondResult, "Invalid node type");
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() == 2, "Invalid number of arguments for an assignment inside an if/else operation");
        CPPADCG_ASSERT_KNOWN(node.getArguments()[0].getOperation() != nullptr, "Invalid argument for an an assignment inside an if/else operation");
        CPPADCG_ASSERT_KNOWN(node.getArguments()[1].getOperation() != nullptr, "Invalid argument for an an assignment inside an if/else operation");

        // just follow the argument
        Node& nodeArg = *node.getArguments()[1].getOperation();
        printAssigment(nodeArg);
    }

    virtual void printUserCustom(Node& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::UserCustom, "Invalid node type");

        throw CGException("Unable to generate C source code for user custom operation nodes.");
    }

    inline bool isDependent(const Node& arg) const {
        if (arg.getOperationType() == CGOpCode::LoopIndexedDep) {
            return true;
        }
        size_t id = getVariableID(arg);
        return id > _independentSize && id < _minTemporaryVarID;
    }

    virtual void printParameter(const Base& value) {
        // make sure all digits of floating point values are printed
        std::ostringstream os;
        os << std::setprecision(_parameterPrecision) << value;

        std::string number = os.str();
        _code << number;

        if (std::abs(value) > Base(0) && value != Base(1) && value != Base(-1)) {
            if (number.find('.') == std::string::npos && number.find('e') == std::string::npos) {
                // also make sure there is always a '.' after the number in
                // order to avoid integer overflows
                _code << '.';
            }
        }
    }

    virtual const std::string& getComparison(enum CGOpCode op) const {
        switch (op) {
            case CGOpCode::ComLt:
                return _C_COMP_OP_LT;

            case CGOpCode::ComLe:
                return _C_COMP_OP_LE;

            case CGOpCode::ComEq:
                return _C_COMP_OP_EQ;

            case CGOpCode::ComGe:
                return _C_COMP_OP_GE;

            case CGOpCode::ComGt:
                return _C_COMP_OP_GT;

            case CGOpCode::ComNe:
                return _C_COMP_OP_NE;

            default:
                CPPAD_ASSERT_UNKNOWN(0);
        }
        throw CGException("Invalid comparison operator code"); // should never get here
    }

    inline const std::string& getPrintfBaseFormat() {
        static const std::string format; // empty string
        return format;
    }

    static bool isFunction(enum CGOpCode op) {
        return isUnaryFunction(op) || op == CGOpCode::Pow;
    }

    static bool isUnaryFunction(enum CGOpCode op) {
        switch (op) {
            case CGOpCode::Abs:
            case CGOpCode::Acos:
            case CGOpCode::Asin:
            case CGOpCode::Atan:
            case CGOpCode::Cosh:
            case CGOpCode::Cos:
            case CGOpCode::Exp:
            case CGOpCode::Log:
            case CGOpCode::Sinh:
            case CGOpCode::Sin:
            case CGOpCode::Sqrt:
            case CGOpCode::Tanh:
            case CGOpCode::Tan:
#if CPPAD_USE_CPLUSPLUS_2011
            case CGOpCode::Erf:
            case CGOpCode::Asinh:
            case CGOpCode::Acosh:
            case CGOpCode::Atanh:
            case CGOpCode::Expm1:
            case CGOpCode::Log1p:
#endif
                return true;
            default:
                return false;
        }
    }

    inline static bool isCondAssign(enum CGOpCode op) {
        switch (op) {
            case CGOpCode::ComLt:
            case CGOpCode::ComLe:
            case CGOpCode::ComEq:
            case CGOpCode::ComGe:
            case CGOpCode::ComGt:
            case CGOpCode::ComNe:
                return true;
            default:
                return false;
        }
    }
private:

    class AtomicFuncArray {
    public:
        std::string data;
        unsigned long size;
        bool sparse;
        size_t idx_id;
        unsigned long nnz;
        unsigned short scope;
    };
};
template<class Base>
const std::string LanguageC<Base>::U_INDEX_TYPE = "unsigned long";

template<class Base>
const std::string LanguageC<Base>::_C_COMP_OP_LT = "<";
template<class Base>
const std::string LanguageC<Base>::_C_COMP_OP_LE = "<=";
template<class Base>
const std::string LanguageC<Base>::_C_COMP_OP_EQ = "==";
template<class Base>
const std::string LanguageC<Base>::_C_COMP_OP_GE = ">=";
template<class Base>
const std::string LanguageC<Base>::_C_COMP_OP_GT = ">";
template<class Base>
const std::string LanguageC<Base>::_C_COMP_OP_NE = "!=";

template<class Base>
const std::string LanguageC<Base>::_C_STATIC_INDEX_ARRAY = "index";

template<class Base>
const std::string LanguageC<Base>::_C_SPARSE_INDEX_ARRAY = "idx";

template<class Base>
const std::string LanguageC<Base>::_ATOMIC_TX = "atx";

template<class Base>
const std::string LanguageC<Base>::_ATOMIC_TY = "aty";

template<class Base>
const std::string LanguageC<Base>::_ATOMIC_PX = "apx";

template<class Base>
const std::string LanguageC<Base>::_ATOMIC_PY = "apy";

template<class Base>
const std::string LanguageC<Base>::ATOMICFUN_STRUCT_DEFINITION = "typedef struct Array {\n"
"    void* data;\n"
"    " + U_INDEX_TYPE + " size;\n"
"    int sparse;\n"
"    const " + U_INDEX_TYPE + "* idx;\n"
"    " + U_INDEX_TYPE + " nnz;\n"
"} Array;\n"
"\n"
"struct LangCAtomicFun {\n"
"    void* libModel;\n"
"    int (*forward)(void* libModel, int atomicIndex, int q, int p, const Array tx[], Array* ty);\n"
"    int (*reverse)(void* libModel, int atomicIndex, int p, const Array tx[], Array* px, const Array py[]);\n"
"};";

} // END cg namespace
} // END CppAD namespace

#endif