#include "pbd/Context.h"

#include <pybind11/pybind11.h>

#include <string>
#include <iostream>

namespace py = pybind11;


PYBIND11_MODULE(py_pbd_simulation, m) 
{
    // TODO Add additionnal binding here for Particle and Vec2 ! 

    py::class_<Context>(m, "Context")
        .def(py::init<int>())
        .def("num_particles", &Context::num_particles)
        .def("updatePhysicalSystem", &Context::updatePhysicalSystem);
}

