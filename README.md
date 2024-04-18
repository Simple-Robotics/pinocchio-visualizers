# pinocchio-visualizers

This package aims to provide basic functionality and common interfaces for visualizers (in C++/Python) for the [Pinocchio](https://github.com/stack-of-tasks/pinocchio) rigid-body dynamics library.

## Dependencies

- [Pinocchio](https://github.com/stack-of-tasks/pinocchio), of course (built with hpp-fcl support)
- A C++17 compliant compiler

## Usage in a CMake project

After acquiring the target, link to it:

```cmake
target_link_libraries(mytarget [PUBLIC|PRIVATE|INTERFACE] pinocchio-visualizers::pinocchio-visualizers)
```

## Installation

### As a submodule

In your own repository, you can add it as a submodule:

```bash
git submodule add https://github.com/ManifoldFR/pinocchio-visualizers
```

and in CMake, add it as a subdirectory before using the target:

```cmake
add_subdirectory(pinocchio-visualizers)
```

### Installing from source with CMake

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --target install
```
