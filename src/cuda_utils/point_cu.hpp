#ifndef POINT_CU_HPP__
#define POINT_CU_HPP__


#include <stdio.h>
#include <stdlib.h>

#include "cuda_compiler_interop.hpp"

#include "vec3_cu.hpp"

/// Do not change these
#define XAXIS (0)
#define YAXIS (1)
#define ZAXIS (2)

struct Point_cu {

    float x, y, z;

    IF_CUDA_DEVICE_HOST
    Point_cu(){
        x = 0.f; y = 0.f; z = 0.f;
    }

    IF_CUDA_DEVICE_HOST
    Point_cu(float a, float b, float c){
        x = a; y = b; z = c;
    }

    IF_CUDA_DEVICE_HOST
    inline void set(float x_, float y_, float z_) {
        x = x_; y = y_; z = z_;
    }

    static Point_cu random(float r){
        float r2 = 2.f * r;
        float x_ = rand() * 1.f /RAND_MAX;
        float y_ = rand() * 1.f /RAND_MAX;
        float z_ = rand() * 1.f /RAND_MAX;
        return Point_cu(x_ * r2 - r, y_ * r2 - r, z_ * r2 - r);
    }

    /// displacement
    IF_CUDA_DEVICE_HOST
    Point_cu operator+(const Vec3_cu &v_) const {
        return Point_cu(x+v_.x, y+v_.y, z+v_.z);
    }

    /// displacement
    IF_CUDA_DEVICE_HOST
    Point_cu operator-(const Vec3_cu &v_) const {
        return Point_cu(x-v_.x, y-v_.y, z-v_.z);
    }

    /// difference
    IF_CUDA_DEVICE_HOST
    Vec3_cu operator-(const Point_cu &p_) const {
        return Vec3_cu(x-p_.x, y-p_.y, z-p_.z);
    }

    /// opposite point
    IF_CUDA_DEVICE_HOST
    Point_cu operator-() const {
        return Point_cu(-x, -y, -z);
    }

    IF_CUDA_DEVICE_HOST
    Point_cu operator/(float s) const {
        return Point_cu(x/s, y/s, z/s);
    }

    /// squared distance to another point
    IF_CUDA_DEVICE_HOST
    float distance_squared (const Point_cu& p_) const {
        return (p_ - *this).norm_squared();
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

    // TODO: supr this and use explicit function like to_vector()
    IF_CUDA_DEVICE_HOST
    operator Vec3_cu() const {
        return Vec3_cu(x, y, z);
    }

    #ifdef __CUDACC__
    __device__ __host__
    float4 to_float4() const{
        return make_float4(x, y, z, 0.f);
    }
    #endif

    IF_CUDA_DEVICE_HOST
    Vec3_cu to_vector() const {
        return Vec3_cu(x, y, z);
    }

    IF_CUDA_DEVICE_HOST
    inline Point_cu operator+(const Point_cu& p) const {
        return Point_cu(x + p.x, y + p.y, z + p.z);
    }
    IF_CUDA_DEVICE_HOST
    inline Point_cu operator*(const Point_cu& p) const {
        return Point_cu(x * p.x, y * p.y, z * p.z);
    }

    IF_CUDA_DEVICE_HOST
    inline Point_cu operator*(float f) const {
        return Point_cu(x * f, y * f, z * f);
    }

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

    IF_CUDA_DEVICE_HOST
    inline Point_cu perm_x() const{
        return Point_cu(x, y, z);
    }

    IF_CUDA_DEVICE_HOST
    inline Point_cu perm_y() const{
        return Point_cu(y, z, x);
    }

    IF_CUDA_DEVICE_HOST
    inline Point_cu perm_z() const{
        return Point_cu(z, x, y);
    }

    inline void print() const {
        printf("(%f,%f,%f) ", x, y, z);
    }
};

IF_CUDA_DEVICE_HOST
inline Point_cu Vec3_cu::proj_on_plane(const Point_cu& pos_plane,
                                       const Point_cu& to_project) const
{
    return to_project + (*this) * (pos_plane - to_project).dot( (*this) );
}

IF_CUDA_DEVICE_HOST
inline Point_cu Vec3_cu::to_point() const{
    return Point_cu(x, y, z);
}


#endif // POINT_CU_HPP__
