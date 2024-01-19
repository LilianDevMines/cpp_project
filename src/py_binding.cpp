#include "pbd/Context.h"

#include <pybind11/pybind11.h>

#include <string>
#include <iostream>

namespace py = pybind11;


PYBIND11_MODULE(py_pbd_simulation, m) 
{
    // TODO Add additionnal binding here for Particle and Vec2 ! 
    py::class_<Vec2>(m,"Vec2")
        .def(py::init<float,float>())
        .def_readonly("x",&Vec2::x)
        .def_readonly("y",&Vec2::y);

    py::class_<Particle>(m,"Particle")
        .def_readonly("position", &Particle::position)
        .def_readonly("radius",&Particle::radius)
        .def_readonly("draw_id",&Particle::draw_id);

    py::class_<Context>(m, "Context")
        .def(py::init<int>())
        .def("num_particles", &Context::num_particles)
        .def("updatePhysicalSystem", &Context::updatePhysicalSystem)
        .def("addParticle", &Context::addParticle)
        .def("particle", &Context::particle)
        .def("addPlan", &Context::addPlan)
        .def("addWater", &Context::addWater);
}       

