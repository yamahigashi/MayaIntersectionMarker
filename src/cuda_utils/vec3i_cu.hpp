#ifndef VEC3I_CU_HPP__
#define VEC3I_CU_HPP__

#include "cuda_compiler_interop.hpp"
#include "math_cu.hpp"
#include <stdlib.h>
#include <stdio.h>
#include "vec3_cu.hpp"

/// Do not change these
#define XAXIS (0)
#define YAXIS (1)
#define ZAXIS (2)


/** @brief Vector type compatible GCC and NVCC

  Vec3i_cu can be used at your convenience with either NVCC or GCC.

  @note: the overloading operator '*' between vector is not defined on purpose.
  There is too many different behavior for the same signature. this could lead
  too a lot of errors as one would expect the operator to do a component wise
  multiplication or a cross product or a scalar product...
  You may use dot() to do a scalar product. cross() for cross product and mult()
  for component wise multiplication.

*/

struct Point_cu;

// =============================================================================
struct Vec3i_cu {
// =============================================================================

    int x, y, z;

    // -------------------------------------------------------------------------
    /// @name Constructors
    // -------------------------------------------------------------------------

    IF_CUDA_DEVICE_HOST
    Vec3i_cu() { x = 0; y = 0; z = 0; }

    IF_CUDA_DEVICE_HOST
    Vec3i_cu(int x_, int y_, int z_) { x = x_; y = y_; z = z_; }

    IF_CUDA_DEVICE_HOST
    static inline Vec3i_cu unit_x(){
        return Vec3i_cu(1, 0, 0);
    }

    IF_CUDA_DEVICE_HOST
    static inline Vec3i_cu unit_y(){
        return Vec3i_cu(0, 1, 0);
    }

    IF_CUDA_DEVICE_HOST
    static inline Vec3i_cu unit_z(){
        return Vec3i_cu(0, 0, 1);
    }

    IF_CUDA_DEVICE_HOST
    static inline Vec3i_cu zero() {
        return Vec3i_cu(0, 0, 0);
    }

    IF_CUDA_DEVICE_HOST
    static inline Vec3i_cu unit_scale(){
        return Vec3i_cu(1, 1, 1);
    }

    IF_CUDA_DEVICE_HOST
    inline void set(int x_, int y_, int z_) {
        x = x_; y = y_; z = z_;
    }

    #ifdef __CUDACC__
    __device__ __host__
    int4 to_int4() const{
        return make_int4(x, y, z, 0);
    }
    #endif

    static Vec3i_cu random(int r){
        float r2 = 2.f * (float)r;
        float x_ = rand() * 1.f / (float)RAND_MAX;
        float y_ = rand() * 1.f / (float)RAND_MAX;
        float z_ = rand() * 1.f / (float)RAND_MAX;
        return Vec3i_cu( (int)(x_ * r2 - r),
                         (int)(y_ * r2 - r),
                         (int)(z_ * r2 - r));
    }

    // -------------------------------------------------------------------------
    /// @name Overload operators
    // -------------------------------------------------------------------------

    /// addition
    IF_CUDA_DEVICE_HOST
    Vec3i_cu operator+(const Vec3i_cu &v_) const {
        return Vec3i_cu(x+v_.x, y+v_.y, z+v_.z);
    }

    IF_CUDA_DEVICE_HOST
    Vec3i_cu operator*(const Vec3i_cu &v_) const {
        return Vec3i_cu(x*v_.x, y*v_.y, z*v_.z);
    }

    IF_CUDA_DEVICE_HOST
    Vec3i_cu& operator+= (const Vec3i_cu &v_) {
        x += v_.x;
        y += v_.y;
        z += v_.z;
        return *this;
    }

    IF_CUDA_DEVICE_HOST
    bool operator!= (const Vec3i_cu &v_) const {
        return (x != v_.x) |  (y != v_.y) | (z != v_.z);
    }

    IF_CUDA_DEVICE_HOST
    Vec3i_cu operator+(int f_) const {
        return Vec3i_cu(x+f_, y+f_, z+f_);
    }

    IF_CUDA_DEVICE_HOST
    Vec3i_cu& operator+= (int f_) {
        x += f_;
        y += f_;
        z += f_;
        return *this;
    }

    /// substraction
    IF_CUDA_DEVICE_HOST
    Vec3i_cu operator-(const Vec3i_cu &v_) const {
        return Vec3i_cu(x-v_.x, y-v_.y, z-v_.z);
    }

    /// opposite vector
    IF_CUDA_DEVICE_HOST
    Vec3i_cu operator-() const {
        return Vec3i_cu(-x, -y, -z);
    }

    /// scalar multiplication
    IF_CUDA_DEVICE_HOST
    Vec3i_cu operator*(const int d_) const {
        return Vec3i_cu(x*d_, y*d_, z*d_);
    }

    IF_CUDA_DEVICE_HOST
    Vec3i_cu operator/(const int d_) const {
        return Vec3i_cu(x/d_, y/d_, z/d_);
    }

    IF_CUDA_DEVICE_HOST
    Vec3i_cu& operator/=(const int d_) {
        x /= d_;
        y /= d_;
        z /= d_;
        return *this;
    }

    IF_CUDA_DEVICE_HOST
    Vec3i_cu& operator*=(const int d_) {
        x *= d_;
        y *= d_;
        z *= d_;
        return *this;
    }

    IF_CUDA_DEVICE_HOST
    Vec3i_cu& operator*=(const Vec3i_cu& d_) {
        x *= d_.x;
        y *= d_.y;
        z *= d_.z;
        return *this;
    }

    /// Convert to floating point vector
    IF_CUDA_DEVICE_HOST
    operator Vec3_cu(){
        return Vec3_cu((float)x, (float)y, (float)z);
    }

    IF_CUDA_DEVICE_HOST
    bool operator==(const Vec3i_cu& d_)  const {
        return x == d_.x && y == d_.y && z == d_.z;
    }

    // -------------------------------------------------------------------------
    /// @name Operators on vector
    // -------------------------------------------------------------------------

    /// product of all components
    IF_CUDA_DEVICE_HOST
    int product() const { return x*y*z; }

    /// sum of all components
    IF_CUDA_DEVICE_HOST
    int sum() const { return x+y+z; }

    /// semi dot product
    IF_CUDA_DEVICE_HOST
    Vec3i_cu mult(const Vec3i_cu& v) const {
        return Vec3i_cu(x*v.x, y*v.y, z*v.z);
    }

    /// cross product
    IF_CUDA_DEVICE_HOST
    Vec3i_cu cross(const Vec3i_cu& v_) const {
        return Vec3i_cu(y*v_.z-z*v_.y, z*v_.x-x*v_.z, x*v_.y-y*v_.x);
    }


    /// dot product
    IF_CUDA_DEVICE_HOST
    int dot(const Vec3i_cu& v_) const {
        return x*v_.x+y*v_.y+z*v_.z;
    }

    /// Compute the cotangente (1./tan) between 'this' and v_
    IF_CUDA_DEVICE_HOST
    float cotan(const Vec3i_cu& v_) const {
        // cot(alpha ) = dot(v1, v2) / ||cross(v1, v2)||
        // = ||v1||*||v2||*cos( angle(v1, v2) ) / ||v1||*||v2|| * sin( angle(v1, v2) )
        // = cos( angle(v1, v2) ) / sin( angle(v1, v2) )
        // = 1 / tan( angle(v1, v2) )
        // = cot( angle(v1, v2) ) = cot( alpha )
        return  (float)(this->dot(v_)) / (this->cross(v_)).norm();
    }

    /// absolute value of the dot product
    IF_CUDA_DEVICE_HOST
    int abs_dot(const Vec3i_cu& v_) const {
        return abs(x * v_.x + y * v_.y + z * v_.z);
    }

    /// norm squared
    IF_CUDA_DEVICE_HOST
    int norm_squared() const {
        return dot(*this);
    }

    /// norm
    IF_CUDA_DEVICE_HOST
    float norm() const {
        return sqrtf( (float)norm_squared() );
    }

    /// value of the min coordinate
    IF_CUDA_DEVICE_HOST
    int get_min() const {
        return min(min(x,y),z);
    }

    /// value of the max coordinate
    IF_CUDA_DEVICE_HOST
    int get_max() const {
        return max(max(x,y),z);
    }

    /// clamp each vector values
    IF_CUDA_DEVICE_HOST
    Vec3i_cu clamp(int min_v, int max_v) const {
        return Vec3i_cu( min( max(x, min_v), max_v),
                         min( max(y, min_v), max_v),
                         min( max(z, min_v), max_v));
    }

    /// rotate of 0 step to the left (present for symmetry)
    IF_CUDA_DEVICE_HOST
    Vec3i_cu perm_x() const {
        return Vec3i_cu(x, y, z);
    }

    /// rotate of 1 step to the left (so that y is the first coordinate)
    IF_CUDA_DEVICE_HOST
    Vec3i_cu perm_y() const {
        return Vec3i_cu(y, z, x);
    }

    /// rotate of 2 steps to the left (so that z is the first coordinate)
    IF_CUDA_DEVICE_HOST
    Vec3i_cu perm_z() const {
        return Vec3i_cu(z, x, y);
    }

    IF_CUDA_DEVICE_HOST
    inline const int& operator[](int i) const{
        switch(i){
        case XAXIS: return x;
        case YAXIS: return y;
        }
        return z;
    }

    IF_CUDA_DEVICE_HOST
    inline int& operator[](int i){
        switch(i){
        case XAXIS: return x;
        case YAXIS: return y;
        }
        return z;
    }

    void print(){
        printf("%d, %d, %d\n", x, y, z);
    }

};
// =============================================================================

#endif // VEC3I_CU_HPP__
