#pragma once

#include "base-visualizer.hpp"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/model.hpp>

#include <chrono>
#include <thread>

namespace pinocchio_visualizers {

BaseVisualizer::BaseVisualizer(const Model& model,
                               const GeometryModel& visual_model,
                               const GeometryModel* collision_model)
    : m_model(&model),
      m_visualModel(&visual_model),
      m_collisionModel(collision_model),
      data(model),
      visualData(visual_model),
      collisionData() {
  if (m_collisionModel) collisionData = GeometryData(*m_collisionModel);
}

void BaseVisualizer::rebuildData() {
  data = Data(*m_model);
  visualData = GeometryData(*m_visualModel);
  if (m_collisionModel) collisionData = GeometryData(*m_collisionModel);
}

void BaseVisualizer::display(const std::optional<ConstVectorRef>& q) {
  if (q.has_value()) {
    forwardKinematics(*m_model, data, *q);
  }
  updateGeometryPlacements(*m_model, data, *m_visualModel, visualData);
  displayImpl();
}

void BaseVisualizer::play(const std::vector<ConstVectorRef>& qs, Scalar dt) {
  const auto nsteps = qs.size();
  std::chrono::steady_clock clk;
  const auto ms = std::chrono::milliseconds(unsigned(dt * 1e3));

  for (size_t i = 0; i < nsteps; i++) {
    const auto cur = clk.now();
    this->display(qs[i]);
    if (!this->forceRedraw()) return;
    std::this_thread::sleep_until(cur + ms);
  }
}

}  // namespace pinocchio_visualizers
