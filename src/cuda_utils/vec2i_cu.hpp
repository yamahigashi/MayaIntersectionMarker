#ifndef VEC2I_CU_HPP__
#define VEC2I_CU_HPP__

#include "macros.hpp"
#include "math_cu.hpp"
#include "vec2_cu.hpp"
#include <stdlib.h>
#include <stdio.h>

/// Do not change these
#define XAXIS (0)
#define YAXIS (1)

/// @name Vec2i_cu
/// @brief 2D integer vector compatible cuda
// =============================================================================
struct Vec2i_cu {
// =============================================================================

    int x, y;

    // -------------------------------------------------------------------------
    /// @name Constructors
    // -------------------------------------------------------------------------

    IF_CUDA_DEVICE_HOST
    Vec2i_cu() { x = 0; y = 0; }

    IF_CUDA_DEVICE_HOST
    Vec2i_cu(int x_, int y_) { x = x_; y = y_; }

    IF_CUDA_DEVICE_HOST
    static inline Vec2i_cu unit_x(){ return Vec2i_cu(1, 0); }

    IF_CUDA_DEVICE_HOST
    static inline Vec2i_cu unit_y(){ return Vec2i_cu(0, 1); }

    IF_CUDA_DEVICE_HOST
    static inline Vec2i_cu zero() { return Vec2i_cu(0, 0); }

    IF_CUDA_DEVICE_HOST
    static inline Vec2i_cu unit_scale(){ return Vec2i_cu(1, 1); }

    IF_CUDA_DEVICE_HOST
    inline void set(int x_, int y_) { x = x_; y = y_; }

    #ifdef __CUDACC__
    __device__ __host__
    int2 to_int2() const{ return make_int2(x, y); }
    #endif

    static Vec2i_cu random(int r){
        int r2 = 2 * r;
        int x_ = rand();
        int y_ = rand();
        return Vec2i_cu(x_ * r2 - r, y_ * r2 - r);
    }

    // -------------------------------------------------------------------------
    /// @name Overload operators
    // -------------------------------------------------------------------------

    /// addition
    IF_CUDA_DEVICE_HOST
    Vec2i_cu operator+(const Vec2i_cu& v_) const {
        return Vec2i_cu(x+v_.x, y+v_.y);
    }

    IF_CUDA_DEVICE_HOST
    Vec2i_cu operator*(const Vec2i_cu& v_) const {
        return Vec2i_cu(x*v_.x, y*v_.y);
    }

    IF_CUDA_DEVICE_HOST
    Vec2i_cu& operator+= (const Vec2i_cu& v_) {
        x += v_.x;
        y += v_.y;
        return *this;
    }

    IF_CUDA_DEVICE_HOST
    Vec2i_cu& operator-= (const Vec2i_cu& v_) {
        x -= v_.x;
        y -= v_.y;
        return *this;
    }

    IF_CUDA_DEVICE_HOST
    Vec2i_cu operator+(int f_) const {
        return Vec2i_cu(x+f_, y+f_);
    }

    IF_CUDA_DEVICE_HOST
    Vec2i_cu& operator+= (int f_) {
        x += f_;
        y += f_;
        return *this;
    }

    IF_CUDA_DEVICE_HOST
    Vec2i_cu& operator-= (int f_) {
        x -= f_;
        y -= f_;
        return *this;
    }

    /// substraction
    IF_CUDA_DEVICE_HOST
    Vec2i_cu operator-(const Vec2i_cu &v_) const {
        return Vec2i_cu(x-v_.x, y-v_.y);
    }

    /// opposite vector
    IF_CUDA_DEVICE_HOST
    Vec2i_cu operator-() const {
        return Vec2i_cu(-x, -y);
    }

    /// scalar multiplication
    IF_CUDA_DEVICE_HOST
    Vec2i_cu operator*(const int d_) const {
        return Vec2i_cu(x*d_, y*d_);
    }

    IF_CUDA_DEVICE_HOST
    Vec2i_cu operator/(const int d_) const {
        return Vec2i_cu(x/d_, y/d_);
    }

    IF_CUDA_DEVICE_HOST
    Vec2i_cu& operator*=(const int d_) {
        x *= d_;
        y *= d_;
        return *this;
    }

    IF_CUDA_DEVICE_HOST
    Vec2i_cu& operator*=(const Vec2i_cu& d_) {
        x *= d_.x;
        y *= d_.y;
        return *this;
    }

    /// Convert to floating point vector
    IF_CUDA_DEVICE_HOST
    operator Vec2_cu(){
        return Vec2_cu((float)x, (float)y);
    }

    IF_CUDA_DEVICE_HOST
    bool operator==(const Vec2i_cu& d_)  const {
        return x == d_.x && y == d_.y;
    }

    // -------------------------------------------------------------------------
    /// @name Operators on vector
    // -------------------------------------------------------------------------

    /// product of all components
    IF_CUDA_DEVICE_HOST
    int product() const { return x*y; }

    /// sum of all components
    IF_CUDA_DEVICE_HOST
    int sum() const { return x+y; }

    /// semi dot product
    IF_CUDA_DEVICE_HOST
    Vec2i_cu mult(const Vec2i_cu& v) const {
        return Vec2i_cu(x*v.x, y*v.y);
    }

    /// dot product
    IF_CUDA_DEVICE_HOST
    int dot(const Vec2i_cu &v_) const {
        return x * v_.x + y * v_.y;
    }

    /// norm squared
    IF_CUDA_DEVICE_HOST
    float norm_squared() const {
        return (float)dot(*this);
    }

    /// norm
    IF_CUDA_DEVICE_HOST
    float norm() const {
        return sqrtf(norm_squared());
    }

    /// value of the min coordinate
    IF_CUDA_DEVICE_HOST
    int get_min() const {
        return min(x,y);
    }

    /// value of the max coordinate
    IF_CUDA_DEVICE_HOST
    int get_max() const {
        return max(x,y);
    }

    /// clamp each vector values
    IF_CUDA_DEVICE_HOST
    Vec2i_cu clamp(int min_v, int max_v) const {
        return Vec2i_cu( min( max(x, min_v), max_v),
                         min( max(y, min_v), max_v));
    }

    /// rotate of 0 step to the left (present for symmetry)
    IF_CUDA_DEVICE_HOST
    Vec2i_cu perm_x() const {
        return Vec2i_cu(x, y);
    }

    /// rotate of 1 step to the left (so that y is the first coordinate)
    IF_CUDA_DEVICE_HOST
    Vec2i_cu perm_y() const {
        return Vec2i_cu(y, x);
    }

    IF_CUDA_DEVICE_HOST
    inline const int& operator[](int i) const{  return i == XAXIS ? x : y; }

    IF_CUDA_DEVICE_HOST
    inline int& operator[](int i){ return i == XAXIS ? x : y; }

    inline void print(){
        printf("%d, %d\n", x, y);
    }

};
// =============================================================================

#endif // VEC2I_CU_HPP__
