// Tool to debug c extensions with a c debugger.
//
// How to use it:
//
//   1. Write a module in python that calls the extension you want to debug.
//      Make sure the extension code is called at import time.
//
//   2. Build opensfm/cmake_build in Debug mode.
//
//   3. Tell the debugger to run
//
//        ./debug_c_extension module
//
//      where module is the name of the module that you wrote.

#include <pybind11/embed.h>
#include <iostream>
namespace py = pybind11;

int main(int argc, char *argv[]) {
  py::scoped_interpreter guard{};

  if (argc != 2) {
    std::cout << "Usage: " << argv[0] << " module" << std::endl;
  } else {
    py::module sys = py::module::import(argv[1]);
  }
}
