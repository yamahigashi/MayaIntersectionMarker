#ifndef VEC2_CU_HPP__
#define VEC2_CU_HPP__

#include "macros.hpp"
#include "math_cu.hpp"
#include "vec3_cu.hpp"
#include <stdlib.h>
#include <stdio.h>

/// Do not change these
#define XAXIS (0)
#define YAXIS (1)


/** @brief Vector type compatible GCC and NVCC

  Vec2_cu can be used at your convenience with either NVCC or GCC.

  @note: the overloading operator '*' between vector is not defined on purpose.
  There is too many different behavior for the same signature. this could lead
  too a lot of errors as one would expect the operator to do a component wise
  multiplication or a scalar product...
  You may use dot() to do a scalar product and mult() for component wise
  multiplication.
*/

// =============================================================================
struct Vec2_cu {
// =============================================================================

    float x, y;

    // -------------------------------------------------------------------------
    /// @name Constructors
    // -------------------------------------------------------------------------

    IF_CUDA_DEVICE_HOST
    Vec2_cu() { x = 0.f; y = 0.f; }

    IF_CUDA_DEVICE_HOST
    Vec2_cu(float x_, float y_) { x = x_; y = y_; }

    IF_CUDA_DEVICE_HOST
    static inline Vec2_cu unit_x(){ return Vec2_cu(1.f, 0.f); }

    IF_CUDA_DEVICE_HOST
    static inline Vec2_cu unit_y(){ return Vec2_cu(0.f, 1.f); }

    IF_CUDA_DEVICE_HOST
    static inline Vec2_cu zero() { return Vec2_cu(0.f, 0.f); }

    IF_CUDA_DEVICE_HOST
    static inline Vec2_cu unit_scale(){ return Vec2_cu(1.f, 1.f); }

    IF_CUDA_DEVICE_HOST
    inline void set(float x_, float y_) { x = x_; y = y_; }

    #ifdef __CUDACC__
    __device__ __host__
    float2 to_float2() const{ return make_float2(x, y); }
    #endif

    static Vec2_cu random(float r){
        float r2 = 2.f * r;
        float x_ = rand() * 1.f /RAND_MAX;
        float y_ = rand() * 1.f /RAND_MAX;
        return Vec2_cu(x_ * r2 - r, y_ * r2 - r);
    }

    // -------------------------------------------------------------------------
    /// @name Overload operators
    // -------------------------------------------------------------------------

    /// addition
    IF_CUDA_DEVICE_HOST
    Vec2_cu operator+(const Vec2_cu &v_) const {
        return Vec2_cu(x+v_.x, y+v_.y);
    }

    IF_CUDA_DEVICE_HOST
    Vec2_cu operator*(const Vec2_cu &v_) const {
        return Vec2_cu(x*v_.x, y*v_.y);
    }

    IF_CUDA_DEVICE_HOST
    Vec2_cu& operator+= (const Vec2_cu &v_) {
        x += v_.x;
        y += v_.y;
        return *this;
    }

    IF_CUDA_DEVICE_HOST
    Vec2_cu operator+(float f_) const {
        return Vec2_cu(x+f_, y+f_);
    }

    IF_CUDA_DEVICE_HOST
    Vec2_cu& operator+= (float f_) {
        x += f_;
        y += f_;
        return *this;
    }

    /// substraction
    IF_CUDA_DEVICE_HOST
    Vec2_cu operator-(const Vec2_cu &v_) const {
        return Vec2_cu(x-v_.x, y-v_.y);
    }

    /// opposite vector
    IF_CUDA_DEVICE_HOST
    Vec2_cu operator-() const {
        return Vec2_cu(-x, -y);
    }

    /// scalar multiplication
    IF_CUDA_DEVICE_HOST
    Vec2_cu operator*(const float d_) const {
        return Vec2_cu(x*d_, y*d_);
    }

    IF_CUDA_DEVICE_HOST
    Vec2_cu operator/(const float d_) const {
        return Vec2_cu(x/d_, y/d_);
    }

    IF_CUDA_DEVICE_HOST
    Vec2_cu& operator*=(const float d_) {
        x *= d_;
        y *= d_;
        return *this;
    }

    IF_CUDA_DEVICE_HOST
    Vec2_cu& operator*=(const Vec2_cu& d_) {
        x *= d_.x;
        y *= d_.y;
        return *this;
    }

    IF_CUDA_DEVICE_HOST
    Vec2_cu& operator/=(const float d_) {
        x /= d_;
        y /= d_;
        return *this;
    }

    // -------------------------------------------------------------------------
    /// @name Operators on vector
    // -------------------------------------------------------------------------

    /// product of all components
    IF_CUDA_DEVICE_HOST
    float product() const { return x*y; }

    /// sum of all components
    IF_CUDA_DEVICE_HOST
    float sum() const { return x+y; }

    /// semi dot product
    IF_CUDA_DEVICE_HOST
    Vec2_cu mult(const Vec2_cu& v) const {
        return Vec2_cu(x*v.x, y*v.y);
    }

    /// dot product
    IF_CUDA_DEVICE_HOST
    float dot(const Vec2_cu &v_) const {
        return x * v_.x + y * v_.y;
    }

    /// absolute value of the dot product
    IF_CUDA_DEVICE_HOST
    float abs_dot(const Vec2_cu &v_) const {
        return fabsf(x * v_.x + y * v_.y);
    }

    /// norm squared
    IF_CUDA_DEVICE_HOST
    float norm_squared() const {
        return dot(*this);
    }

    /// normalization
    IF_CUDA_DEVICE_HOST
    Vec2_cu normalized() const {
        return (*this) * (1.f/sqrtf(norm_squared()));
    }

    /// normalization
    IF_CUDA_DEVICE_HOST
    float normalize() {
        float l = sqrtf(norm_squared());
        float f = 1.f / l;
        x *= f;
        y *= f;
        return l;
    }

    /// normalization
    IF_CUDA_DEVICE_HOST
    float safe_normalize() {
        float l = sqrtf(norm_squared());
        if(l > 1e-10f){
            float f = 1.f / l;
            x *= f;
            y *= f;
            return l;
        } else {
            x = 1.f;
            y = 0.f;
            return 0.f;
        }
    }

    /// norm
    IF_CUDA_DEVICE_HOST
    float norm() const {
        return sqrtf(norm_squared());
    }

    /// value of the min coordinate
    IF_CUDA_DEVICE_HOST
    float get_min() const {
        return fminf(x,y);
    }

    /// value of the max coordinate
    IF_CUDA_DEVICE_HOST
    float get_max() const {
        return fmaxf(x,y);
    }

    /// clamp each vector values
    IF_CUDA_DEVICE_HOST
    Vec2_cu clamp(float min_v, float max_v) const {
        return Vec2_cu( fminf( fmaxf(x, min_v), max_v),
                        fminf( fmaxf(y, min_v), max_v));
    }

    /// rotate of 0 step to the left (present for symmetry)
    IF_CUDA_DEVICE_HOST
    Vec2_cu perm_x() const {
        return Vec2_cu(x, y);
    }

    /// rotate of 1 step to the left (so that y is the first coordinate)
    IF_CUDA_DEVICE_HOST
    Vec2_cu perm_y() const {
        return Vec2_cu(y, x);
    }

    /// Get a random orthogonal vector
    IF_CUDA_DEVICE_HOST
    Vec2_cu get_ortho() const
    {
        Vec3_cu ortho = Vec3_cu(x, y, 0.f).cross( Vec3_cu(0.f, 0.f, 1.f) );
        return Vec2_cu( ortho.x, ortho.y);
    }

    /// @return the vector to_project projected on the line defined by the
    /// direction '*this'
    /// @warning don't forget to normalize the vector before calling this !
    IF_CUDA_DEVICE_HOST
    Vec2_cu proj_on_line(const Vec2_cu& to_project) const {
        return (*this) * (*this).dot( to_project );
    }

    IF_CUDA_DEVICE_HOST
    inline const float& operator[](int i) const{  return i == XAXIS ? x : y; }

    IF_CUDA_DEVICE_HOST
    inline float& operator[](int i){ return i == XAXIS ? x : y; }

    inline void print(){
        printf("%f, %f\n", x, y);
    }

};
// =============================================================================

#endif // VEC2_CU_HPP__
