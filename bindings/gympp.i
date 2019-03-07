%module gympp

%{
#define SWIG_FILE_WITH_INIT
#include "gympp/Environment.h"
#include "gympp/common.h"
#include "gympp/GymFactory.h"
#include "gympp/gyms/Ignition.h"
#include "gympp/Space.h"

SWIGRUNTIME int
SWIG_Python_TypeErrorOccurred(PyObject *obj)
{
    PyObject *error;
    if (obj)
        return 0;
    error = PyErr_Occurred();
    return error && PyErr_GivenExceptionMatches(error, PyExc_TypeError);
}

SWIGRUNTIME void
SWIG_Python_RaiseOrModifyTypeError(const char *message)
{
    if (SWIG_Python_TypeErrorOccurred(NULL)) {
        /* Use existing TypeError to preserve stacktrace and enhance with given message */
        PyObject *newvalue;
        PyObject *type = NULL, *value = NULL, *traceback = NULL;
        PyErr_Fetch(&type, &value, &traceback);
#if PY_VERSION_HEX >= 0x03000000
        newvalue = PyUnicode_FromFormat("%S\nAdditional information:\n%s", value, message);
#else
        newvalue = PyString_FromFormat("%s\nAdditional information:\n%s", PyString_AsString(value), message);
#endif
        Py_XDECREF(value);
        PyErr_Restore(type, newvalue, traceback);
    } else {
        /* Raise TypeError using given message */
        PyErr_SetString(PyExc_TypeError, message);
    }
}

%}

%naturalvar;

%include <std_string.i>
%include <std_vector.i>

// Convert python list to std::vector
%template(Vector_i) std::vector<int>;
%template(Vector_u) std::vector<size_t>;
%template(Vector_f) std::vector<float>;
%template(Vector_d) std::vector<double>;

%include "gympp/common.h"
%template(BufferContainer_i) gympp::BufferContainer<int>;
%template(BufferContainer_u) gympp::BufferContainer<size_t>;
%template(BufferContainer_f) gympp::BufferContainer<float>;
%template(BufferContainer_d) gympp::BufferContainer<double>;
%template(get_i) gympp::data::Sample::get<int>;
%template(get_u) gympp::data::Sample::get<size_t>;
%template(get_f) gympp::data::Sample::get<float>;
%template(get_d) gympp::data::Sample::get<double>;
%template(getBuffer_i) gympp::data::Sample::getBuffer<int>;
%template(getBuffer_u) gympp::data::Sample::getBuffer<size_t>;
%template(getBuffer_f) gympp::data::Sample::getBuffer<float>;
%template(getBuffer_d) gympp::data::Sample::getBuffer<double>;
%template(RangeFloat) gympp::Range<float>;
%template(RangeDouble) gympp::Range<double>;

%include "optional.i"
%template(Optional_i) std::optional<int>;
%template(Optional_state) std::optional<gympp::State>;
%template(Optional_sample) std::optional<gympp::data::Sample>;

%include <std_shared_ptr.i>
%shared_ptr(gympp::spaces::Space)
%shared_ptr(gympp::spaces::details::TBox<double>)
%shared_ptr(gympp::spaces::Discrete)

%include "gympp/Space.h"
%template(Box_d) gympp::spaces::details::TBox<double>;

%shared_ptr(gympp::Environment)
%include "gympp/Environment.h"
%include "gympp/GymFactory.h"
