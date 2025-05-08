#ifndef VEC3_CU_HPP__
#define VEC3_CU_HPP__

#include "cuda_compiler_interop.hpp"

#include "math_cu.hpp"
#include <stdlib.h>
#include <stdio.h>

/// Do not change these
#define XAXIS (0)
#define YAXIS (1)
#define ZAXIS (2)


/** @brief Vector type compatible GCC and NVCC

  Vec3_cu can be used at your convenience with either NVCC or GCC.

  @note: the overloading operator '*' between vector is not defined on purpose.
  There is too many different behavior for the same signature. this could lead
  too a lot of errors as one would expect the operator to do a component wise
  multiplication or a cross product or a scalar product...
  You may use dot() to do a scalar product. cross() for cross product and mult()
  for component wise multiplication.

*/

struct Point_cu;

// =============================================================================
struct Vec3_cu {
// =============================================================================

    float x, y, z;

    // -------------------------------------------------------------------------
    /// @name Constructors
    // -------------------------------------------------------------------------

    IF_CUDA_DEVICE_HOST
    Vec3_cu() { x = 0.f; y = 0.f; z = 0.f; }

    IF_CUDA_DEVICE_HOST
    Vec3_cu(float x_, float y_, float z_) { x = x_; y = y_; z = z_; }

    IF_CUDA_DEVICE_HOST
    static inline Vec3_cu unit_x(){
        return Vec3_cu(1.f, 0.f, 0.f);
    }

    IF_CUDA_DEVICE_HOST
    static inline Vec3_cu unit_y(){
        return Vec3_cu(0.f, 1.f, 0.f);
    }

    IF_CUDA_DEVICE_HOST
    static inline Vec3_cu unit_z(){
        return Vec3_cu(0.f, 0.f, 1.f);
    }

    IF_CUDA_DEVICE_HOST
    static inline Vec3_cu zero() {
        return Vec3_cu(0.f, 0.f, 0.f);
    }

    IF_CUDA_DEVICE_HOST
    static inline Vec3_cu unit_scale(){
        return Vec3_cu(1.f, 1.f, 1.f);
    }

    IF_CUDA_DEVICE_HOST
    inline void set(float x_, float y_, float z_) {
        x = x_; y = y_; z = z_;
    }

    #ifdef __CUDACC__
    __device__ __host__
    float4 to_float4() const{
        return make_float4(x, y, z, 0.f);
    }
    #endif

    static Vec3_cu random(float r){
        float r2 = 2.f * r;
        float x_ = rand() * 1.f /RAND_MAX;
        float y_ = rand() * 1.f /RAND_MAX;
        float z_ = rand() * 1.f /RAND_MAX;
        return Vec3_cu(x_ * r2 - r, y_ * r2 - r, z_ * r2 - r);
    }

    // -------------------------------------------------------------------------
    /// @name Overload operators
    // -------------------------------------------------------------------------

    /// addition
    IF_CUDA_DEVICE_HOST
    Vec3_cu operator+(const Vec3_cu &v_) const {
        return Vec3_cu(x+v_.x, y+v_.y, z+v_.z);
    }

    IF_CUDA_DEVICE_HOST
    Vec3_cu operator*(const Vec3_cu &v_) const {
        return Vec3_cu(x*v_.x, y*v_.y, z*v_.z);
    }

    IF_CUDA_DEVICE_HOST
    Vec3_cu& operator+= (const Vec3_cu &v_) {
        x += v_.x;
        y += v_.y;
        z += v_.z;
        return *this;
    }

    IF_CUDA_DEVICE_HOST
    bool operator!= (const Vec3_cu &v_) const {
        return (x != v_.x) |  (y != v_.y) | (z != v_.z);
    }

    IF_CUDA_DEVICE_HOST
    Vec3_cu operator+(float f_) const {
        return Vec3_cu(x+f_, y+f_, z+f_);
    }

    IF_CUDA_DEVICE_HOST
    Vec3_cu& operator+= (float f_) {
        x += f_;
        y += f_;
        z += f_;
        return *this;
    }

    /// substraction
    IF_CUDA_DEVICE_HOST
    Vec3_cu operator-(const Vec3_cu &v_) const {
        return Vec3_cu(x-v_.x, y-v_.y, z-v_.z);
    }

    /// opposite vector
    IF_CUDA_DEVICE_HOST
    Vec3_cu operator-() const {
        return Vec3_cu(-x, -y, -z);
    }

    /// scalar multiplication
    IF_CUDA_DEVICE_HOST
    Vec3_cu operator*(const float d_) const {
        return Vec3_cu(x*d_, y*d_, z*d_);
    }

    IF_CUDA_DEVICE_HOST
    Vec3_cu operator/(const float d_) const {
        return Vec3_cu(x/d_, y/d_, z/d_);
    }

    IF_CUDA_DEVICE_HOST
    Vec3_cu& operator/=(const float d_) {
        x /= d_;
        y /= d_;
        z /= d_;
        return *this;
    }

    IF_CUDA_DEVICE_HOST
    Vec3_cu& operator*=(const float d_) {
        x *= d_;
        y *= d_;
        z *= d_;
        return *this;
    }

    IF_CUDA_DEVICE_HOST
    Vec3_cu& operator*=(const Vec3_cu& d_) {
        x *= d_.x;
        y *= d_.y;
        z *= d_.z;
        return *this;
    }

    // -------------------------------------------------------------------------
    /// @name Operators on vector
    // -------------------------------------------------------------------------

    /// product of all components
    IF_CUDA_DEVICE_HOST
    float product() const { return x*y*z; }

    /// sum of all components
    IF_CUDA_DEVICE_HOST
    float sum() const { return x+y+z; }

    /// semi dot product (component wise multiplication)
    IF_CUDA_DEVICE_HOST
    Vec3_cu mult(const Vec3_cu& v) const {
        return Vec3_cu(x*v.x, y*v.y, z*v.z);
    }

    /// component wise division
    IF_CUDA_DEVICE_HOST
    Vec3_cu div(const Vec3_cu& v) const {
        return Vec3_cu(x/v.x, y/v.y, z/v.z);
    }

    /// cross product
    IF_CUDA_DEVICE_HOST
    Vec3_cu cross(const Vec3_cu& v_) const {
        return Vec3_cu(y*v_.z-z*v_.y, z*v_.x-x*v_.z, x*v_.y-y*v_.x);
    }


    /// dot product
    IF_CUDA_DEVICE_HOST
    float dot(const Vec3_cu& v_) const {
        return x*v_.x+y*v_.y+z*v_.z;
    }

    /// Compute the cotangente (1./tan) between 'this' and v_
    IF_CUDA_DEVICE_HOST
    float cotan(const Vec3_cu& v_) const {
        // cot(alpha ) = dot(v1, v2) / ||cross(v1, v2)||
        // = ||v1||*||v2||*cos( angle(v1, v2) ) / ||v1||*||v2|| * sin( angle(v1, v2) )
        // = cos( angle(v1, v2) ) / sin( angle(v1, v2) )
        // = 1 / tan( angle(v1, v2) )
        // = cot( angle(v1, v2) ) = cot( alpha )
        return (this->dot(v_)) / (this->cross(v_)).norm();
    }

    /// absolute value of the dot product
    IF_CUDA_DEVICE_HOST
    float abs_dot(const Vec3_cu& v_) const {
        return fabsf(x * v_.x + y * v_.y + z * v_.z);
    }

    /// norm squared
    IF_CUDA_DEVICE_HOST
    float norm_squared() const {
        return dot(*this);
    }

    /// normalization
    IF_CUDA_DEVICE_HOST
    Vec3_cu normalized() const {
        return (*this) * (1.f/sqrtf(norm_squared()));
    }

    /// normalization
    IF_CUDA_DEVICE_HOST
    float normalize() {
        float l = sqrtf(norm_squared());
        float f = 1.f / l;
        x *= f;
        y *= f;
        z *= f;
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
            z *= f;
            return l;
        } else {
            x = 1.f;
            y = 0.f;
            z = 0.f;
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
        return fminf(fminf(x,y),z);
    }

    /// value of the max coordinate
    IF_CUDA_DEVICE_HOST
    float get_max() const {
        return fmaxf(fmaxf(x,y),z);
    }

    /// clamp each vector values
    IF_CUDA_DEVICE_HOST
    Vec3_cu clamp(float min_v, float max_v) const {
        return Vec3_cu( fminf( fmaxf(x, min_v), max_v),
                        fminf( fmaxf(y, min_v), max_v),
                        fminf( fmaxf(z, min_v), max_v));
    }

    /// rotate of 0 step to the left (present for symmetry)
    IF_CUDA_DEVICE_HOST
    Vec3_cu perm_x() const {
        return Vec3_cu(x, y, z);
    }

    /// rotate of 1 step to the left (so that y is the first coordinate)
    IF_CUDA_DEVICE_HOST
    Vec3_cu perm_y() const {
        return Vec3_cu(y, z, x);
    }

    /// rotate of 2 steps to the left (so that z is the first coordinate)
    IF_CUDA_DEVICE_HOST
    Vec3_cu perm_z() const {
        return Vec3_cu(z, x, y);
    }

    IF_CUDA_DEVICE_HOST
    void coordinate_system (Vec3_cu& v1_, Vec3_cu& v2_) const {
        //for numerical stability, and seen that z will
        //always be present, take the greatest component between
        //x and y.
        if( fabsf(x) > fabsf(y) ) {
            float inv_len = 1.f / sqrtf(x * x + z * z);
            Vec3_cu tmp(-z * inv_len, 0.f, x * inv_len);
            v1_ = tmp;
        } else {
            float inv_len = 1.f / sqrtf (y * y + z * z);
            Vec3_cu tmp(0.f, z * inv_len, -y * inv_len);
            v1_ = tmp;
        }
        v2_ = (*this).cross (v1_);
    }

    /// Get a random orthogonal vector
    IF_CUDA_DEVICE_HOST
    Vec3_cu get_ortho() const
    {
        Vec3_cu ortho = this->cross(Vec3_cu(1.f, 0.f, 0.f));

        if (ortho.norm_squared() < 1e-06f * 1e-06f)
            ortho = this->cross( Vec3_cu(0.f, 1.f, 0.f) );

        return ortho;
    }

    /// @return the vector to_project projected on the plane defined by the
    /// normal '*this'
    /// @warning don't forget to normalize the vector before calling
    /// proj_on_plane() !
    IF_CUDA_DEVICE_HOST
    Vec3_cu proj_on_plane(const Vec3_cu& to_project) const
    {
        return ( (*this).cross(to_project) ).cross( (*this) );
    }

    /// @return the point to_project projected on the plane defined by the
    /// normal '*this' and passing through pos_plane
    /// @warning don't forget to normalize the vector before calling
    /// proj_on_plane() !
    /// @note implemented in Point_cu.h because of cross definitions
    IF_CUDA_DEVICE_HOST
    inline Point_cu proj_on_plane(const Point_cu& pos_plane,
                                  const Point_cu& to_project) const;

    /// @note implemented in Point_cu.h because of cross definitions
    IF_CUDA_DEVICE_HOST
    inline Point_cu to_point() const;

    IF_CUDA_DEVICE_HOST
    inline const float& operator[](int i) const{
        switch(i){
        case XAXIS: return x;
        case YAXIS: return y;
        }
        return z;
    }

    IF_CUDA_DEVICE_HOST
    inline float& operator[](int i){
        switch(i){
        case XAXIS: return x;
        case YAXIS: return y;
        }
        return z;
    }

    void print(){
        printf("%f, %f, %f\n", x, y, z);
    }

};
// =============================================================================

#endif // VEC3_CU_HPP__
