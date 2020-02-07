// Tool to debug c extensions with a c debugger.
//
// How to use it:
//
//   1. Write a function in python that calls the extension you want to debug.
//      Make sure it can be called without arguments.
//
//   2. Build opensfm/cmake_build in Debug mode.
//
//   3. Tell the debugger to run
//
//        ./debug_c_extension path module function
//
//      where path is the path to the module that contains the function
//      that you wrote.

#include "third_party/PythonExtensionPatterns/py_import_call_execute.h"

int main(int argc, const char *argv[]) {
    import_call_execute(argc, argv);
}