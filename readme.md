# FARBIK ik solver

This is a C++ library implementing the FABRIK inverse kinematics solver (see references for paper).

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

The library has no prerequisites.
The visualizer used for development of the library requires irrlicht - a 3D graphics library.

### Using the library in your project

If you are using CMake, you can add the library to your project by including 
```
solver/solver.hpp
```
and adding similar line to your CMakeLists.txt:

```
add_subdirectory(${PROJECT_SOURCE_DIR}/ikSolver/solver)
```
and including 
```
ikSolver
```
in
```
target_link_libraries
```


Otherwise you can build the library by running
```
cd solver && mkdir build && cd build && cmake .. && make
```
and then link libikSolver.a and add the solver/include include directory.

### Development

If you are interested in helping with the development, you can use the included visualizer tool. In that case you will need to install irrlicht.

## Built With

* [CMake](https://cmake.org/) - Dependency Management

## Authors

* **Aleš Koblížek** - *Initial work* - [kobli](https://github.com/kobli)

See also the list of [contributors](https://github.com/kobli/ikSolver/graphs/contributors) who participated in this project.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## References

- Andreas Aristidou and Joan Lasenby. FABRIK: A fast, iterative solver for the inverse kinematics problem. Graph. Models, 73(5):243–260, September 2011.

