#include <pybind11/pybind11.h>

#include <string>
#include <iostream>

namespace py = pybind11;

struct Value {
    float x;
};

struct Dog {
    Dog() = default;
    Dog(const std::string& _name) 
    : name(_name)
    {
        std::cout << "BUILDING" << "\n";
    }

    std::string name;

    std::string bark() const {
        return name + ": woof!";
    }
};

int add(int a, int b) { return a + b; }

PYBIND11_MODULE(my_ext, m) {
    m.def("add", &add);

    m.attr("the_answer") = 42;

    py::class_<Dog>(m, "Dog")
        .def(py::init<>())
        .def(py::init<const std::string &>())
        .def("bark", &Dog::bark)
        .def_readwrite("name", &Dog::name);

    py::class_<Value>(m, "Value")
        .def(py::init<float>())
        .def_readonly("x", &Value::x);
}


