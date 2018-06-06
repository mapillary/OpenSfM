//
//  py_import_call_execute.h
//  PythonSubclassList
//
//  Created by Paul Ross on 10/06/2016.
//  Copyright (c) 2016 Paul Ross. All rights reserved.
//

#ifndef __PythonSubclassList__py_import_call_execute__
#define __PythonSubclassList__py_import_call_execute__

#ifdef __cplusplus
extern "C" {
#endif
    
/** This imports a Python module and calls a specific function in it.
 * It's arguments are similar to main():
 * argc - Number of strings in argv
 * argv - Expected to be 4 strings:
 *      - Name of the executable.
 *      - Path to the directory that the Python module is in.
 *      - Name of the Python module.
 *      - Name of the function in the module.
 *
 * The Python interpreter will be initialised and the path to the Python module
 * will be added to sys.paths then the module will be imported.
 * The function will be called with no arguments and its return value will be
 * ignored.
 *
 * This returns 0 on success, non-zero on failure.
 */
int import_call_execute(int argc, const char *argv[]);

    
#ifdef __cplusplus
} // extern "C"
#endif

#endif /* defined(__PythonSubclassList__py_import_call_execute__) */
