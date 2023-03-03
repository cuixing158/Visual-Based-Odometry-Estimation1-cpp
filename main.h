///
/// @file           : main.h
/// @target         : Texas Instruments->C6000
/// @details        : for path build map algorithms(loop+pose)
/// @author         : cuixingxing
/// @email          : xingxing.cui@long-horn.com
/// @date           : 02-Mar-2023 10:06:28
/// @version        : V0.1.2
/// @copyright      : Copyright (C) 2023 Long-Horn Inc.All rights reserved.
///

/*************************************************************************/
/* This automatically generated example C++ main file shows how to call  */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

#ifndef MAIN_H
#define MAIN_H

/// @include file    : Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

/// Type Declarations
namespace buildMapping {
class HDMapping;

}

/// Function Declarations
extern int main(int argc, char **argv);

extern void main_constructWorldMap(buildMapping::HDMapping *instancePtr);

#endif
///
/// File trailer for main.h
///
/// [EOF]
///
