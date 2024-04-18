#pragma once

#include "../base-visualizer.hpp"

#include <eigenpy/eigenpy.hpp>

namespace pinocchio_visualizers {
namespace bp = boost::python;

// user-defined literal
inline bp::arg operator""_a(const char *argname, std::size_t) {
  return bp::arg(argname);
}

template <class Visualizer>
struct VisualizerVisitor : bp::def_visitor<VisualizerVisitor<Visualizer>> {
  static void setCameraPose_proxy(Visualizer &vis,
                                  const Eigen::Matrix4d &pose) {
    vis.setCameraPose(pose);
  }

  static void setCameraPose_proxy2(Visualizer &vis, const pin::SE3 &pose) {
    vis.setCameraPose(pose);
  }

  template <class... PyArgs>
  void visit(bp::class_<PyArgs...> &cl) const {
    cl.def("initViewer", &Visualizer::initViewer)
        .def("loadViewerModel", &Visualizer::loadViewerModel)
        .def("rebuildData", &Visualizer::rebuildData)
        .def("display", &Visualizer::display, ("self"_a, "q"_a = std::nullopt))
        .def("setCameraTarget", &Visualizer::setCameraTarget,
             ("self"_a, "target"))
        .def("setCameraPosition", &Visualizer::setCameraPosition,
             ("self"_a, "position"))
        .def("setCameraPose", setCameraPose_proxy, ("self"_a, "pose"))
        .def("setCameraZoom", setCameraPose_proxy2, ("self"_a, "value"))
        .def("clean", &Visualizer::clean, ("self"_a))
        .add_property("model",
                      bp::make_function(&Visualizer::model,
                                        bp::return_internal_reference<>()))
        .add_property("visualModel",
                      bp::make_function(&Visualizer::visualModel,
                                        bp::return_internal_reference<>()))
        .add_property("collisionModel",
                      bp::make_function(&Visualizer::collisionModel,
                                        bp::return_internal_reference<>()))
        .def_readwrite("data", &Visualizer::data)
        .def_readwrite("visualData", &Visualizer::visualData)
        .def_readwrite("collisionData", &Visualizer::collisionData);
  }
};

}  // namespace pinocchio_visualizers
