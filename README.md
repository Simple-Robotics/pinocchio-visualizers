# pinocchio-visualizers

This package aims to provide basic functionality and common interfaces for visualizers (in C++/Python) for the [Pinocchio](https://github.com/stack-of-tasks/pinocchio) rigid-body dynamics library.

## Dependencies

- [Pinocchio](https://github.com/stack-of-tasks/pinocchio), of course (built with hpp-fcl support)
- A C++17 compliant compiler

## Installing from source

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --target install
```
