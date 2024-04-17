#include "pinocchio-visualizers/base-visualizer.hpp"

#include <type_traits>


using namespace pinocchio_visualizers;

static_assert(std::is_abstract_v<BaseVisualizer<double>>);

int main()
{
  return 1;
}
