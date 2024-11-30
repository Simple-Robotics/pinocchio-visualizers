# pinocchio-visualizers

This package aims to provide basic functionality and common interfaces for visualizers (in C++/Python) for the [Pinocchio](https://github.com/stack-of-tasks/pinocchio) rigid-body dynamics library.

## Dependencies

- [Pinocchio](https://github.com/stack-of-tasks/pinocchio), of course (built with ~~HPP-FCL~~ Coal support)
- A C++17 compliant compiler

## Usage in your source files

In exactly **one** file in your application (or its target dependencies), include the main header with the definition to pull in the implementation:

```cpp
#define PINOCCHIO_VISUALIZERS_IMPLEMENTATION
#include <pinocchio-visualizers/base-visualizer.hpp>
```

## Usage in a CMake project

### Installed

The library (sources and CMake target files) can be installed following:

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --target install
```

Then, the CMake targets can be found through `find_package(pinocchio-visualizers)`.
After acquiring the target, link to it:

```cmake
target_link_libraries(mytarget [PUBLIC|PRIVATE|INTERFACE] pinocchio-visualizers::pinocchio-visualizers)
```

### As a submodule

In your own repository, you can add it as a submodule:

```bash
git submodule add https://github.com/Simple-Robotics/pinocchio-visualizers
```

and in CMake, add it as a subdirectory before using the target:

```cmake
add_subdirectory(pinocchio-visualizers)
```
