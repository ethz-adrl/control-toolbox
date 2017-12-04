// derived from
// Copyright (c) 2015-2016, ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the ETH Zurich, Wyss Zurich, Zurich Eye nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL ETH Zurich, Wyss Zurich, Zurich Eye BE LIABLE
// FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Derived from work of Benno Evers licensed:
//
// The MIT License (MIT)
//
// Copyright (c) 2014 Benno Evers
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <ct/core/plot/plot.h>

#define CHECK_EQ(arg1, arg2)                                                                                          \
    if (arg1 != arg2)                                                                                                 \
        throw std::runtime_error("dimension mismatch, #arg1 (" + std::to_string(arg1) + ") is not equal to #arg2 (" + \
                                 std::to_string(arg2) + ")");

#ifdef PLOTTING_ENABLED
#include <Python.h>
#endif

namespace ct {
namespace core {
namespace plot {

#ifdef PLOTTING_ENABLED
namespace detail {

// -----------------------------------------------------------------------------
struct _interpreter
{
    PyObject* s_python_function_show;
    PyObject* s_python_function_save;
    PyObject* s_python_function_figure;
    PyObject* s_python_function_plot;
    PyObject* s_python_function_subplot;
    PyObject* s_python_function_hist;
    PyObject* s_python_function_boxplot;
    PyObject* s_python_function_legend;
    PyObject* s_python_function_xlim;
    PyObject* s_python_function_ylim;
    PyObject* s_python_function_title;
    PyObject* s_python_function_axis;
    PyObject* s_python_function_xlabel;
    PyObject* s_python_function_ylabel;
    PyObject* s_python_function_ion;
    PyObject* s_python_function_grid;
    PyObject* s_python_empty_tuple;

    /** For now, _interpreter is implemented as a singleton since its currently
   * not possible to have multiple independent embedded python interpreters
   * without patching the python source code or starting a seperate process
   * for each.
   * http://bytes.com/topic/python/answers/793370-multiple-independent-python-interpreters-c-c-program
   */

    static _interpreter& get()
    {
        static _interpreter ctx;
        return ctx;
    }

private:
    _interpreter()
    {
        char name[] = "plotting";  // silence compiler warning about const strings
        Py_SetProgramName(name);   // optional but recommended
        Py_Initialize();

        PyObject* pyplotname = PyString_FromString("matplotlib.pyplot");
        PyObject* pylabname = PyString_FromString("pylab");
        if (!pyplotname || !pylabname)
        {
            throw std::runtime_error("couldnt create string");
        }

        PyObject* pymod = PyImport_Import(pyplotname);
        Py_DECREF(pyplotname);
        if (!pymod)
        {
            throw std::runtime_error("Error loading module matplotlib.pyplot!");
        }

        PyObject* pylabmod = PyImport_Import(pylabname);
        Py_DECREF(pylabname);
        if (!pymod)
        {
            throw std::runtime_error("Error loading module pylab!");
        }

        s_python_function_show = PyObject_GetAttrString(pymod, "show");
        s_python_function_figure = PyObject_GetAttrString(pymod, "figure");
        s_python_function_plot = PyObject_GetAttrString(pymod, "plot");
        s_python_function_subplot = PyObject_GetAttrString(pymod, "subplot");
        s_python_function_hist = PyObject_GetAttrString(pymod, "hist");
        s_python_function_boxplot = PyObject_GetAttrString(pymod, "boxplot");
        s_python_function_legend = PyObject_GetAttrString(pymod, "legend");
        s_python_function_ylim = PyObject_GetAttrString(pymod, "ylim");
        s_python_function_title = PyObject_GetAttrString(pymod, "title");
        s_python_function_axis = PyObject_GetAttrString(pymod, "axis");
        s_python_function_xlabel = PyObject_GetAttrString(pymod, "xlabel");
        s_python_function_ylabel = PyObject_GetAttrString(pymod, "ylabel");
        s_python_function_grid = PyObject_GetAttrString(pymod, "grid");
        s_python_function_xlim = PyObject_GetAttrString(pymod, "xlim");
        s_python_function_ion = PyObject_GetAttrString(pymod, "ion");

        s_python_function_save = PyObject_GetAttrString(pylabmod, "savefig");

        if (!s_python_function_show || !s_python_function_save || !s_python_function_figure ||
            !s_python_function_plot || !s_python_function_subplot || !s_python_function_hist ||
            !s_python_function_boxplot || !s_python_function_legend || !s_python_function_xlim ||
            !s_python_function_ylim || !s_python_function_title || !s_python_function_axis ||
            !s_python_function_xlabel || !s_python_function_ylabel || !s_python_function_ion || !s_python_function_grid)
        {
            throw std::runtime_error("Couldnt find required function!");
        }

        if (!PyFunction_Check(s_python_function_show) || !PyFunction_Check(s_python_function_save) ||
            !PyFunction_Check(s_python_function_figure) || !PyFunction_Check(s_python_function_plot) ||
            !PyFunction_Check(s_python_function_subplot) || !PyFunction_Check(s_python_function_hist) ||
            !PyFunction_Check(s_python_function_boxplot) || !PyFunction_Check(s_python_function_legend) ||
            !PyFunction_Check(s_python_function_xlim) || !PyFunction_Check(s_python_function_ylim) ||
            !PyFunction_Check(s_python_function_title) || !PyFunction_Check(s_python_function_axis) ||
            !PyFunction_Check(s_python_function_xlabel) || !PyFunction_Check(s_python_function_ylabel) ||
            !PyFunction_Check(s_python_function_ion) || !PyFunction_Check(s_python_function_grid))
        {
            throw std::runtime_error("Python object is unexpectedly not a PyFunction.");
        }

        s_python_empty_tuple = PyTuple_New(0);
    }

    ~_interpreter() { Py_Finalize(); }
};
}  // namespace detail

// -----------------------------------------------------------------------------
bool ion()
{
    PyObject* args = PyTuple_New(0);
    PyObject* res = PyObject_CallObject(detail::_interpreter::get().s_python_function_ion, args);

    if (res)
    {
        Py_DECREF(res);
    }

    return res;
}

// -----------------------------------------------------------------------------
//! Create a new figure
bool figure(std::string i)
{
    PyObject* args;
    if (i != "")
    {
        args = PyTuple_New(1);
        PyObject* i_py = PyString_FromString(i.c_str());
        PyTuple_SetItem(args, 0, i_py);
    }
    else
    {
        args = PyTuple_New(0);
    }

    PyObject* res = PyObject_CallObject(detail::_interpreter::get().s_python_function_figure, args);

    if (res)
    {
        Py_DECREF(res);
    }

    return res;
}

// -----------------------------------------------------------------------------
bool hist(const Eigen::Ref<const Eigen::VectorXd>& x, const double bins, const std::string histtype)
{
    // using python lists
    PyObject* xlist = PyList_New(x.size());

    for (int i = 0; i < x.size(); ++i)
    {
        PyList_SetItem(xlist, i, PyFloat_FromDouble(x(i)));
    }

    PyObject* bins_py = PyFloat_FromDouble(bins);
    PyObject* histtype_py = PyString_FromString(histtype.c_str());

    // construct positional args
    PyObject* args = PyTuple_New(8);
    PyTuple_SetItem(args, 0, xlist);
    PyTuple_SetItem(args, 1, bins_py);
    PyTuple_SetItem(args, 2, Py_None);
    PyTuple_SetItem(args, 3, Py_False);
    PyTuple_SetItem(args, 4, Py_None);
    PyTuple_SetItem(args, 5, Py_False);
    PyTuple_SetItem(args, 6, Py_None);
    PyTuple_SetItem(args, 7, histtype_py);

    PyObject* res = PyObject_CallObject(detail::_interpreter::get().s_python_function_hist, args);

    Py_DECREF(xlist);
    Py_DECREF(bins_py);
    Py_DECREF(histtype_py);
    if (res)
    {
        Py_DECREF(res);
    }

    return res;
}

// -----------------------------------------------------------------------------
bool boxplot(const Eigen::Ref<const Eigen::MatrixXd>& x, const std::vector<std::string>& labels)
{
    CHECK_EQ(x.rows(), static_cast<int>(labels.size()));

    // using python lists
    PyObject* data = PyList_New(x.rows());
    PyObject* py_labels = PyList_New(x.rows());

    for (int i = 0; i < x.rows(); ++i)
    {
        PyObject* row = PyList_New(x.cols());

        PyList_SetItem(py_labels, i, PyString_FromString(labels[i].c_str()));

        for (int j = 0; j < x.cols(); ++j)
        {
            PyList_SetItem(row, j, PyFloat_FromDouble(x(i, j)));
        }
        PyList_SetItem(data, i, row);
    }

    // construct positional args
    PyObject* args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, data);

    // construct keyword args
    PyObject* kwargs = PyDict_New();
    PyDict_SetItemString(kwargs, "labels", py_labels);

    PyObject* res = PyObject_Call(detail::_interpreter::get().s_python_function_boxplot, args, kwargs);

    Py_DECREF(data);
    if (res)
    {
        Py_DECREF(res);
    }

    return res;
}

// -----------------------------------------------------------------------------
bool boxplot(const Eigen::Ref<const Eigen::MatrixXd>& x, std::initializer_list<const std::string> labels)
{
    std::vector<std::string> labels_vector;
    labels_vector.insert(labels_vector.end(), labels.begin(), labels.end());

    return boxplot(x, labels_vector);
}

// -----------------------------------------------------------------------------
bool boxplot(const Eigen::Ref<const Eigen::MatrixXd>& x)
{
    std::vector<std::string> labels;
    for (int i = 0; i < x.rows(); ++i)
    {
        labels.push_back(std::to_string(i));
    }
    return boxplot(x, labels);
}

// -----------------------------------------------------------------------------
bool subplot(const size_t nrows, const size_t ncols, const size_t plot_number)
{
    PyObject* nrows_py = PyFloat_FromDouble(nrows);
    PyObject* ncols_py = PyFloat_FromDouble(ncols);
    PyObject* plot_number_py = PyFloat_FromDouble(plot_number);
    PyObject* subplot_args = PyTuple_New(3);
    PyTuple_SetItem(subplot_args, 0, nrows_py);
    PyTuple_SetItem(subplot_args, 1, ncols_py);
    PyTuple_SetItem(subplot_args, 2, plot_number_py);

    PyObject* res = PyObject_CallObject(detail::_interpreter::get().s_python_function_subplot, subplot_args);

    Py_DECREF(nrows_py);
    Py_DECREF(ncols_py);
    Py_DECREF(plot_number_py);
    if (res)
    {
        Py_DECREF(res);
    }

    return res;
}

// -----------------------------------------------------------------------------
bool plot(const Eigen::Ref<const Eigen::MatrixXd>& x_raw,
    const Eigen::Ref<const Eigen::MatrixXd>& y_raw,
    const std::map<std::string, std::string>& keywords)
{
    CHECK_EQ(true, (x_raw.cols() == 1) || (x_raw.rows()) == 1);
    CHECK_EQ(true, (y_raw.cols() == 1) || (y_raw.rows()) == 1);

    Eigen::Map<const Eigen::VectorXd> x(x_raw.data(), x_raw.rows() == 1 ? x_raw.cols() : x_raw.rows());
    Eigen::Map<const Eigen::VectorXd> y(y_raw.data(), y_raw.rows() == 1 ? y_raw.cols() : y_raw.rows());

    CHECK_EQ(true, (x.size() == y.size()));

    // using python lists
    PyObject* xlist = PyList_New(x.size());
    PyObject* ylist = PyList_New(y.size());

    for (int i = 0; i < x.size(); ++i)
    {
        PyList_SetItem(xlist, i, PyFloat_FromDouble(x(i)));
        PyList_SetItem(ylist, i, PyFloat_FromDouble(y(i)));
    }

    // construct positional args
    PyObject* args = PyTuple_New(2);
    PyTuple_SetItem(args, 0, xlist);
    PyTuple_SetItem(args, 1, ylist);

    Py_DECREF(xlist);
    Py_DECREF(ylist);

    // construct keyword args
    PyObject* kwargs = PyDict_New();
    for (std::map<std::string, std::string>::const_iterator it = keywords.begin(); it != keywords.end(); ++it)
    {
        PyDict_SetItemString(kwargs, it->first.c_str(), PyString_FromString(it->second.c_str()));
    }

    PyObject* res = PyObject_Call(detail::_interpreter::get().s_python_function_plot, args, kwargs);

    Py_DECREF(args);
    Py_DECREF(kwargs);
    if (res)
    {
        Py_DECREF(res);
    }

    return res;
}

// -----------------------------------------------------------------------------
bool plot(const Eigen::Ref<const Eigen::MatrixXd>& x_raw,
    const Eigen::Ref<const Eigen::MatrixXd>& y_raw,
    const std::string& s)
{
    CHECK_EQ(true, (x_raw.cols() == 1) || (x_raw.rows() == 1));
    CHECK_EQ(true, (y_raw.cols() == 1) || (y_raw.rows() == 1));

    Eigen::Map<const Eigen::VectorXd> x(x_raw.data(), x_raw.rows() == 1 ? x_raw.cols() : x_raw.rows());
    Eigen::Map<const Eigen::VectorXd> y(y_raw.data(), y_raw.rows() == 1 ? y_raw.cols() : y_raw.rows());

    CHECK_EQ(true, (x.size() == y.size()));

    PyObject* xlist = PyList_New(x.size());
    PyObject* ylist = PyList_New(y.size());
    PyObject* pystring = PyString_FromString(s.c_str());

    for (int i = 0; i < x.size(); ++i)
    {
        PyList_SetItem(xlist, i, PyFloat_FromDouble(x(i)));
        PyList_SetItem(ylist, i, PyFloat_FromDouble(y(i)));
    }

    PyObject* plot_args = PyTuple_New(3);
    PyTuple_SetItem(plot_args, 0, xlist);
    PyTuple_SetItem(plot_args, 1, ylist);
    PyTuple_SetItem(plot_args, 2, pystring);

    PyObject* res = PyObject_CallObject(detail::_interpreter::get().s_python_function_plot, plot_args);

    Py_DECREF(xlist);
    Py_DECREF(ylist);
    Py_DECREF(plot_args);
    if (res)
    {
        Py_DECREF(res);
    }

    return res;
}

// -----------------------------------------------------------------------------
bool labelPlot(const std::string& name,
    const Eigen::Ref<const Eigen::MatrixXd>& x_raw,
    const Eigen::Ref<const Eigen::MatrixXd>& y_raw,
    const std::string& format)
{
    CHECK_EQ(true, (x_raw.cols() == 1) || (x_raw.rows() == 1));
    CHECK_EQ(true, (y_raw.cols() == 1) || (y_raw.rows() == 1));

    Eigen::Map<const Eigen::VectorXd> x(x_raw.data(), x_raw.rows() == 1 ? x_raw.cols() : x_raw.rows());
    Eigen::Map<const Eigen::VectorXd> y(y_raw.data(), y_raw.rows() == 1 ? y_raw.cols() : y_raw.rows());

    CHECK_EQ(true, (x.size() == y.size()));

    PyObject* kwargs = PyDict_New();
    PyDict_SetItemString(kwargs, "label", PyString_FromString(name.c_str()));

    PyObject* xlist = PyList_New(x.size());
    PyObject* ylist = PyList_New(y.size());
    PyObject* pystring = PyString_FromString(format.c_str());

    for (int i = 0; i < x.size(); ++i)
    {
        PyObject* f_xi = PyFloat_FromDouble(x(i));
        PyObject* f_yi = PyFloat_FromDouble(y(i));
        if (!f_xi || !f_yi)
        {
            //      VLOG(1) << "MPL: value could not be converted to PyFloat:" << x(i) << ", "
            //              << y(i);
            continue;
        }
        PyList_SetItem(xlist, i, PyFloat_FromDouble(x(i)));
        PyList_SetItem(ylist, i, PyFloat_FromDouble(y(i)));
    }

    PyObject* plot_args = PyTuple_New(3);
    PyTuple_SetItem(plot_args, 0, xlist);
    PyTuple_SetItem(plot_args, 1, ylist);
    PyTuple_SetItem(plot_args, 2, pystring);

    PyObject* res = PyObject_Call(detail::_interpreter::get().s_python_function_plot, plot_args, kwargs);

    Py_DECREF(kwargs);
    Py_DECREF(xlist);
    Py_DECREF(ylist);
    Py_DECREF(plot_args);
    if (res)
    {
        Py_DECREF(res);
    }

    return res;
}

// -----------------------------------------------------------------------------
bool labelPlot(const std::string& name, const Eigen::Ref<const Eigen::MatrixXd>& y, const std::string& format)
{
    Eigen::Matrix<double, Eigen::Dynamic, 1> x(y.size());
    for (int i = 0; i < x.size(); ++i)
    {
        x(i) = i;
    }
    return labelPlot(name, x, y, format);
}

// -----------------------------------------------------------------------------
bool plot(const Eigen::Ref<const Eigen::MatrixXd>& y, const std::string& format)
{
    Eigen::Matrix<double, Eigen::Dynamic, 1> x(y.size());
    for (int i = 0; i < x.size(); ++i)
    {
        x(i) = i;
    }
    return plot(x, y, format);
}


// -----------------------------------------------------------------------------
bool plot(const std::vector<double>& x,
    const Eigen::Ref<const Eigen::MatrixXd>& y,
    const std::map<std::string, std::string>& keywords)
{
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 1>> x_e(x.data(), x.size());
    return plot(x_e, y, keywords);
}

// -----------------------------------------------------------------------------
bool plot(const std::vector<double>& x, const Eigen::Ref<const Eigen::MatrixXd>& y, const std::string& s)
{
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 1>> x_e(x.data(), x.size());
    return plot(x_e, y, s);
}

// -----------------------------------------------------------------------------
bool labelPlot(const std::string& name,
    const std::vector<double>& x,
    const std::vector<double>& y,
    const std::string& format)
{
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 1>> x_e(x.data(), x.size());
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 1>> y_e(y.data(), y.size());
    return labelPlot(name, x_e, y_e, format);
}

// -----------------------------------------------------------------------------
bool labelPlot(const std::string& name,
    const std::vector<double>& x,
    const Eigen::Ref<const Eigen::MatrixXd>& y,
    const std::string& format)
{
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 1>> x_e(x.data(), x.size());
    return labelPlot(name, x_e, y, format);
}

// -----------------------------------------------------------------------------
void legend()
{
    PyObject* res = PyObject_CallObject(
        detail::_interpreter::get().s_python_function_legend, detail::_interpreter::get().s_python_empty_tuple);

    if (!res)
    {
        throw std::runtime_error("Call to legend() failed.");
    }

    Py_DECREF(res);
}

// -----------------------------------------------------------------------------
void ylim(double ymin, double ymax)
{
    PyObject* list = PyList_New(2);
    PyList_SetItem(list, 0, PyFloat_FromDouble(ymin));
    PyList_SetItem(list, 1, PyFloat_FromDouble(ymax));

    PyObject* args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, list);

    PyObject* res = PyObject_CallObject(detail::_interpreter::get().s_python_function_ylim, args);
    if (!res)
    {
        throw std::runtime_error("Call to ylim() failed.");
    }

    Py_DECREF(list);
    Py_DECREF(args);
    Py_DECREF(res);
}

// -----------------------------------------------------------------------------
void xlim(double xmin, double xmax)
{
    PyObject* list = PyList_New(2);
    PyList_SetItem(list, 0, PyFloat_FromDouble(xmin));
    PyList_SetItem(list, 1, PyFloat_FromDouble(xmax));

    PyObject* args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, list);

    PyObject* res = PyObject_CallObject(detail::_interpreter::get().s_python_function_xlim, args);
    if (!res)
    {
        throw std::runtime_error("Call to xlim() failed.");
    }

    Py_DECREF(list);
    Py_DECREF(args);
    Py_DECREF(res);
}

// -----------------------------------------------------------------------------
void title(const std::string& titlestr)
{
    PyObject* pytitlestr = PyString_FromString(titlestr.c_str());
    PyObject* args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, pytitlestr);

    PyObject* res = PyObject_CallObject(detail::_interpreter::get().s_python_function_title, args);
    if (!res)
    {
        throw std::runtime_error("Call to title() failed.");
    }

    // if PyDeCRFF, the function doesn't work on Mac OS
}

// -----------------------------------------------------------------------------
void axis(const std::string& axisstr)
{
    PyObject* str = PyString_FromString(axisstr.c_str());
    PyObject* args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, str);

    PyObject* res = PyObject_CallObject(detail::_interpreter::get().s_python_function_axis, args);
    if (!res)
    {
        throw std::runtime_error("Call to title() failed.");
    }

    // if PyDeCRFF, the function doesn't work on Mac OS
}

// -----------------------------------------------------------------------------
void xlabel(const std::string& str)
{
    PyObject* pystr = PyString_FromString(str.c_str());
    PyObject* args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, pystr);

    PyObject* res = PyObject_CallObject(detail::_interpreter::get().s_python_function_xlabel, args);
    if (!res)
    {
        throw std::runtime_error("Call to xlabel() failed.");
    }

    // if PyDeCRFF, the function doesn't work on Mac OS
}

// -----------------------------------------------------------------------------
void ylabel(const std::string& str)
{
    PyObject* pystr = PyString_FromString(str.c_str());
    PyObject* args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, pystr);

    PyObject* res = PyObject_CallObject(detail::_interpreter::get().s_python_function_ylabel, args);
    if (!res)
    {
        throw std::runtime_error("Call to ylabel() failed.");
    }

    // if PyDeCRFF, the function doesn't work on Mac OS
}

// -----------------------------------------------------------------------------
void grid(bool flag)
{
    PyObject* pyflag = flag ? Py_True : Py_False;

    PyObject* args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, pyflag);

    PyObject* res = PyObject_CallObject(detail::_interpreter::get().s_python_function_grid, args);
    if (!res)
    {
        throw std::runtime_error("Call to grid() failed.");
    }

    // if PyDeCRFF, the function doesn't work on Mac OS
}

// -----------------------------------------------------------------------------
void show(bool block)
{
    PyObject* pyflag = block ? Py_True : Py_False;

    PyObject* kwargs = PyDict_New();

    PyDict_SetItemString(kwargs, "block", pyflag);

    PyObject* res = PyObject_Call(
        detail::_interpreter::get().s_python_function_show, detail::_interpreter::get().s_python_empty_tuple, kwargs);
    if (!res)
    {
        throw std::runtime_error("Call to show() failed.");
    }

    Py_DECREF(res);
    Py_DECREF(kwargs);
}

// -----------------------------------------------------------------------------
void save(const std::string& filename)
{
    PyObject* pyfilename = PyString_FromString(filename.c_str());

    PyObject* args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, pyfilename);

    PyObject* res = PyObject_CallObject(detail::_interpreter::get().s_python_function_save, args);
    if (!res)
    {
        throw std::runtime_error("Call to save() failed.");
    }

    Py_DECREF(pyfilename);
    Py_DECREF(args);
    Py_DECREF(res);
}

void warn()
{
}

#else

void warn()
{
    std::cout << "CT Core was built without Python bindings. Therefore, plotting will not work" << std::endl;
}

// -----------------------------------------------------------------------------
bool ion()
{
    warn();
    return false;
}

// -----------------------------------------------------------------------------
//! Create a new figure
bool figure(std::string i)
{
    warn();
    return false;
}

// -----------------------------------------------------------------------------
bool hist(const Eigen::Ref<const Eigen::VectorXd>& x, const double bins, const std::string histtype)
{
    warn();
    return false;
}

// -----------------------------------------------------------------------------
bool boxplot(const Eigen::Ref<const Eigen::MatrixXd>& x, const std::vector<std::string>& labels)
{
    warn();
    return false;
}

// -----------------------------------------------------------------------------
bool boxplot(const Eigen::Ref<const Eigen::MatrixXd>& x, std::initializer_list<const std::string> labels)
{
    warn();
    return false;
}

// -----------------------------------------------------------------------------
bool boxplot(const Eigen::Ref<const Eigen::MatrixXd>& x)
{
    warn();
    return false;
}

// -----------------------------------------------------------------------------
bool subplot(const size_t nrows, const size_t ncols, const size_t plot_number)
{
    warn();
    return false;
}

// -----------------------------------------------------------------------------
bool plot(const Eigen::Ref<const Eigen::MatrixXd>& x_raw,
    const Eigen::Ref<const Eigen::MatrixXd>& y_raw,
    const std::map<std::string, std::string>& keywords)
{
    warn();
    return false;
}

// -----------------------------------------------------------------------------
bool plot(const Eigen::Ref<const Eigen::MatrixXd>& x_raw,
    const Eigen::Ref<const Eigen::MatrixXd>& y_raw,
    const std::string& s)
{
    warn();
    return false;
}

// -----------------------------------------------------------------------------
bool labelPlot(const std::string& name,
    const Eigen::Ref<const Eigen::MatrixXd>& x_raw,
    const Eigen::Ref<const Eigen::MatrixXd>& y_raw,
    const std::string& format)
{
    warn();
    return false;
}

// -----------------------------------------------------------------------------
bool labelPlot(const std::string& name, const Eigen::Ref<const Eigen::MatrixXd>& y, const std::string& format)
{
    warn();
    return false;
}

// -----------------------------------------------------------------------------
bool plot(const Eigen::Ref<const Eigen::MatrixXd>& y, const std::string& format)
{
    warn();
    return false;
}


// -----------------------------------------------------------------------------
bool plot(const std::vector<double>& x,
    const Eigen::Ref<const Eigen::MatrixXd>& y,
    const std::map<std::string, std::string>& keywords)
{
    warn();
    return false;
}

// -----------------------------------------------------------------------------
bool plot(const std::vector<double>& x, const Eigen::Ref<const Eigen::MatrixXd>& y, const std::string& s)
{
    warn();
    return false;
}

// -----------------------------------------------------------------------------
bool labelPlot(const std::string& name,
    const std::vector<double>& x,
    const std::vector<double>& y,
    const std::string& format)
{
    warn();
    return false;
}

// -----------------------------------------------------------------------------
bool labelPlot(const std::string& name,
    const std::vector<double>& x,
    const Eigen::Ref<const Eigen::MatrixXd>& y,
    const std::string& format)
{
    warn();
    return false;
}

// -----------------------------------------------------------------------------
void legend()
{
    warn();
}

// -----------------------------------------------------------------------------
void ylim(double ymin, double ymax)
{
    warn();
}

// -----------------------------------------------------------------------------
void xlim(double xmin, double xmax)
{
    warn();
}

// -----------------------------------------------------------------------------
void title(const std::string& titlestr)
{
    warn();
}

// -----------------------------------------------------------------------------
void axis(const std::string& axisstr)
{
    warn();
}

// -----------------------------------------------------------------------------
void xlabel(const std::string& str)
{
    warn();
}

// -----------------------------------------------------------------------------
void ylabel(const std::string& str)
{
    warn();
}

// -----------------------------------------------------------------------------
void grid(bool flag)
{
    warn();
}

// -----------------------------------------------------------------------------
void show(bool block)
{
    warn();
}

// -----------------------------------------------------------------------------
void save(const std::string& filename)
{
    warn();
}
#endif
}  // namespace plot
}  // namespace core
}  // namespace ct

#undef CHECK_EQ
