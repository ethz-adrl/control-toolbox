#ifndef CPPAD_CG_ARRAY_WRAPPER_INCLUDED
#define CPPAD_CG_ARRAY_WRAPPER_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2016 Ciengis
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

# include <cstddef>
# include <iostream>
# include <limits>

namespace CppAD {
namespace cg {

/**
 * A simple wrapper for C arrays.
 * It does not own the data array.
 */
template<class Type>
class ArrayWrapper {
public:
    typedef Type value_type;
    typedef value_type* pointer;
    typedef const value_type* const_pointer;
    typedef value_type& reference;
    typedef const Type& const_reference;
    typedef value_type* iterator;
    typedef const value_type* const_iterator;
    typedef std::reverse_iterator<const_iterator> const_reverse_iterator;
    typedef std::reverse_iterator<iterator> reverse_iterator;
    typedef size_t size_type;
    typedef ptrdiff_t difference_type;
private:
    /**
     * The externally created array
     */
    pointer _data;
    /**
     * The number of elements in the array
     */
    size_type _length;
public:
    /**
     * Default empty constructor for arrays with no elements
     */
    inline ArrayWrapper() :
            _data(nullptr),
            _length(0) {
    }

    /**
     * Creates a wrapper for an existing array.
     *
     * @param array pointer to the first element of the array
     * @param n  size of the array
     */
    inline ArrayWrapper(pointer array,
                        size_type n) :
            _data(array),
            _length(n) {
    }

    /**
     * Creates a wrapper from a vector.
     * It is expected that the vector is not resized while using this wrapper.
     *
     * @param vector the vector to wrap
     */
    inline ArrayWrapper(std::vector<value_type>& vector) :
            _data(vector.data()),
            _length(vector.size()) {
    }

    /**
     * Creates a wrapper from a vector.
     * It is expected that the vector is not resized while using this wrapper.
     *
     * @param vector the vector to wrap
     */
    inline ArrayWrapper(CppAD::vector<value_type>& vector) :
            _data(vector.data()),
            _length(vector.size()) {
    }

    /**
     * Creates a wrapper from a vector.
     * It is expected that the vector is not resized while using this wrapper.
     *
     * @param vector the vector to wrap
     */
    template<class TT = Type>
    inline ArrayWrapper(const std::vector<typename std::remove_const<value_type>::type>& vector,
                        typename std::enable_if<std::is_const<TT>::value>::type* = 0) :
            _data(vector.data()),
            _length(vector.size()) {
    }

    /**
     * Creates a wrapper from a vector.
     * It is expected that the vector is not resized while using this wrapper.
     *
     * @param vector the vector to wrap
     */
    template<class TT = Type>
    inline ArrayWrapper(const CppAD::vector<typename std::remove_const<value_type>::type>& vector,
                        typename std::enable_if<std::is_const<TT>::value>::type* = 0) :
            _data(vector.data()),
            _length(vector.size()) {
    }

    /**
     * Copy constructor
     * @param x
     */
    inline ArrayWrapper(const ArrayWrapper& x) = default;

    /**
     * Desctructor
     */
    virtual ~ArrayWrapper() = default;

    /**
     * @return number of elements in the array.
     */
    inline size_t size() const noexcept {
        return _length;
    }

    /**
     * @return number of elements in the array.
     */
    inline size_type max_size() const noexcept {
        return _length;
    }

    inline bool empty() const noexcept {
        return size() == 0;
    }

    /**
     * @return raw pointer to the array
     */
    inline pointer data() noexcept {
        return _data;
    }

    /**
     * @return raw pointer to the array
     */
    inline const_pointer data() const noexcept {
        return _data;
    }

    inline void fill(const value_type& u) {
        std::fill_n(begin(), size(), u);
    }

    inline void swap(ArrayWrapper& other) noexcept {
        std::swap(other._data, _data);
        std::swap(other._length, _length);
    }

    // Iterators.
    inline iterator begin() noexcept {
        return iterator(data());
    }

    inline const_iterator begin() const noexcept {
        return const_iterator(data());
    }

    inline iterator end() noexcept {
        return iterator(data() + size());
    }

    inline const_iterator end() const noexcept {
        return const_iterator(data() + size());
    }

    inline reverse_iterator rbegin() noexcept {
        return reverse_iterator(end());
    }

    inline const_reverse_iterator rbegin() const noexcept {
        return const_reverse_iterator(end());
    }

    inline reverse_iterator rend() noexcept {
        return reverse_iterator(begin());
    }

    inline const_reverse_iterator rend() const noexcept {
        return const_reverse_iterator(begin());
    }

    inline const_iterator cbegin() const noexcept {
        return const_iterator(data());
    }

    inline const_iterator cend() const noexcept {
        return const_iterator(data() + size());
    }

    inline const_reverse_iterator crbegin() const noexcept {
        return const_reverse_iterator(end());
    }

    inline const_reverse_iterator crend() const noexcept {
        return const_reverse_iterator(begin());
    }

    // Element access.
    inline reference operator[](size_type i) {
        CPPAD_ASSERT_KNOWN(i < size(), "ArrayWrapper: index greater than or equal array size");
        return _data[i];
    }

    inline const_reference operator[](size_type i) const {
        CPPADCG_ASSERT_KNOWN(i < size(), "ArrayWrapper: index greater than or equal array size");
        return _data[i];
    }

    inline reference at(size_type i) {
        if (i >= size())
            throw CGException("ArrayWrapper::at() index ", i, " is greater than or equal array size ", size());
        return _data[i];
    }

    inline const_reference at(size_type i) const {
        if (i >= size())
            throw CGException("ArrayWrapper::at() index ", i, " is greater than or equal array size ", size());
        return _data[i];
    }

    inline reference front() {
        CPPADCG_ASSERT_KNOWN(!empty(), "ArrayWrapper: cannot call front for an empty array");
        return *begin();
    }

    inline const_reference front() const {
        CPPADCG_ASSERT_KNOWN(!empty(), "ArrayWrapper: cannot call front for an empty array");
        return _data[0];
    }

    inline reference back() {
        CPPADCG_ASSERT_KNOWN(!empty(), "ArrayWrapper: cannot call back for an empty array");
        return *(end() - 1);
    }

    inline const_reference back() const {
        CPPADCG_ASSERT_KNOWN(!empty(), "ArrayWrapper: cannot call back for an empty array");
        return _data[size() - 1];
    }

public:
    inline ArrayWrapper& operator=(const ArrayWrapper& x) {
        if (&x == this)
            return *this;

        if (x.size() != size())
            throw CGException("ArrayWrapper: assigning an array with different size: the left hand side array has the size ", size(),
                              " while the right hand side array has the size ", x.size(), ".");

        for(size_t i = 0; i < _length; ++i) {
            _data[i] = x._data[i];
        }

        return *this;
    }

    inline ArrayWrapper& operator=(const std::vector<Type>& x) {
        if (x.size() != size())
            throw CGException("ArrayWrapper: assigning an array with different size: the left hand side array has the size ", size(),
                              " while the right hand side array has the size ", x.size(), ".");

        for(size_t i = 0; i < _length; ++i) {
            _data[i] = x[i];
        }

        return *this;
    }

    inline ArrayWrapper& operator=(const CppAD::vector<Type>& x) {
        if (x.size() != size())
            throw CGException("ArrayWrapper: assigning an array with different size: the left hand side array has the size ", size(),
                              " while the right hand side array has the size ", x.size(), ".");

        for(size_t i = 0; i < _length; ++i) {
            _data[i] = x[i];
        }

        return *this;
    }

};

/**
 * ArrayWrapper output.
 *
 * @param os  stream to write the vector to
 * @param array  array that is output
 * @return the original stream
 */
template<class Type>
inline std::ostream& operator<<(std::ostream& os,
                                const ArrayWrapper<Type>& array) {
    size_t i = 0;
    size_t n = array.size();

    os << "{ ";
    while (i < n) {
        os << array[i++];
        if (i < n)
            os << ", ";
    }
    os << " }";
    return os;
}

} // END cg namespace
} // END CppAD namespace

# endif
