#ifndef MATH_CU_HPP__
#define MATH_CU_HPP__

/** @file math_cu.hpp

  This header allows to use maths functions in both and nvcc and gcc compiler

*/


#ifdef __CUDACC__

#include <cuda_runtime.h>

#else

#include <cmath>
inline float fmaxf(float a, float b){ return a > b ? a : b; }

inline float fminf(float a, float b){ return a < b ? a : b; }

inline int max(int a, int b){ return a > b ? a : b; }

inline int min(int a, int b){ return a < b ? a : b; }
#endif


#endif // MATH_CU_HPP__
