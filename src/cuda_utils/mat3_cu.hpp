#ifndef MAT3_CU_HPP__
#define MAT3_CU_HPP__

#include "cuda_compiler_interop.hpp"
#include "vec3_cu.hpp"
#include "math_cu.hpp"
#include <stdlib.h>
#include <stdio.h>

/**
 * @name Mat3_cu
 * @brief Handling 3*3 matrix
 *
 * @see Transfo Vec3_cu
 */
struct Mat3_cu {

    float a, b, c; ///< first row
    float d, e, f; ///< second row
    float g, h ,i; ///< third row

    IF_CUDA_DEVICE_HOST inline Mat3_cu() {   }

    IF_CUDA_DEVICE_HOST inline
    Mat3_cu(float a_, float b_, float c_,
            float d_, float e_, float f_,
            float g_, float h_, float i_)
    {
        a = a_; b = b_; c = c_;
        d = d_; e = e_; f = f_;
        g = g_; h = h_; i = i_;
    }

    IF_CUDA_DEVICE_HOST inline
    Mat3_cu(const Vec3_cu& x,
            const Vec3_cu& y,
            const Vec3_cu& z)
    {
        a = x.x; b = y.x; c = z.x;
        d = x.y; e = y.y; f = z.y;
        g = x.z; h = y.z; i = z.z;
    }

    IF_CUDA_DEVICE_HOST
    Vec3_cu operator*(const Vec3_cu& v) const
    {
        float x = v.x * a + v.y * b + v.z * c;
        float y = v.x * d + v.y * e + v.z * f;
        float z = v.x * g + v.y * h + v.z * i;
        return Vec3_cu(x, y, z);
    }

    IF_CUDA_DEVICE_HOST
    inline Mat3_cu operator*(const Mat3_cu& m) const
    {
        return Mat3_cu(a * m.a + b * m.d + c * m.g,
                       a * m.b + b * m.e + c * m.h,
                       a * m.c + b * m.f + c * m.i,
                       d * m.a + e * m.d + f * m.g,
                       d * m.b + e * m.e + f * m.h,
                       d * m.c + e * m.f + f * m.i,
                       g * m.a + h * m.d + i * m.g,
                       g * m.b + h * m.e + i * m.h,
                       g * m.c + h * m.f + i * m.i);
    }

    IF_CUDA_DEVICE_HOST
    inline Mat3_cu operator*(float x) const
    {
        return Mat3_cu(a * x, b * x, c * x,
                       d * x, e * x, f * x,
                       g * x, h * x, i * x);
    }


    IF_CUDA_DEVICE_HOST
    inline Mat3_cu operator+(const Mat3_cu& m) const
    {
        return Mat3_cu(a + m.a, b + m.b, c + m.c,
                       d + m.d, e + m.e, f + m.f,
                       g + m.g, h + m.h, i + m.i);
    }


    IF_CUDA_DEVICE_HOST
    inline Mat3_cu operator-(const Mat3_cu& m) const
    {
        return Mat3_cu(a - m.a, b - m.b, c - m.c,
                       d - m.d, e - m.e, f - m.f,
                       g - m.g, h - m.h, i - m.i);
    }

    IF_CUDA_DEVICE_HOST
    inline float det() const
    {
        return a * ( e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
    }

    /// @return the matrix with normalized x, y, z column vectors
    IF_CUDA_DEVICE_HOST
    inline Mat3_cu normalized() const {
        return Mat3_cu(x().normalized(), y().normalized(), z().normalized());
    }

    IF_CUDA_DEVICE_HOST
    inline Mat3_cu inverse() const
    {
        float c0 = e * i - f * h;
        float c1 = f * g - d * i;
        float c2 = d * h - e * g;
        float idet = 1.f / (a * c0 + b * c1 + c * c2);
        return Mat3_cu(c0 , c * h - b * i, b * f - c * e,
                       c1 , a * i - c * g, c * d - a * f,
                       c2 , b * g - a * h, a * e - b * d) * idet;
    }

    IF_CUDA_DEVICE_HOST
    inline Mat3_cu transpose() const
    {
        return Mat3_cu(a, d, g, b, e, h, c, f, i);
    }

    IF_CUDA_DEVICE_HOST
    inline void set_abs()
    {
        a = fabs(a); b = fabs(b); c = fabs(c);
        d = fabs(d); e = fabs(e); f = fabs(f);
        g = fabs(g); h = fabs(h); i = fabs(i);
    }


    IF_CUDA_DEVICE_HOST
    inline float max_elt() const
    {
        return fmaxf(i, fmaxf(fmaxf(fmaxf(a,b),fmaxf(c,d)),
                              fmaxf(fmaxf(e,f),fmaxf(g,h))));
    }

    IF_CUDA_DEVICE_HOST
    inline float min_elt() const
    {
        return fminf(i, fminf(fminf(fminf(a,b),fminf(c,d)),
                              fminf(fminf(e,f),fminf(g,h))));
    }

    IF_CUDA_DEVICE_HOST
    Mat3_cu get_ortho() const
    {
        Mat3_cu h0 = (*this);
        Mat3_cu h1 = h0;
        h1.set_abs();
        float eps =(1.f +  h1.min_elt()) * 1e-5f;
        for(int i = 0; i < 500/* to avoid infinite loop */; i++){
            h0 = (h0 + (h0.inverse()).transpose()) * 0.5f;
            h1 = h1 - h0;
            h1.set_abs();
            if(h1.max_elt() <= eps)
                break;
            h1 = h0;
        }
        return h0;
    }

    IF_CUDA_DEVICE_HOST
    float get_rotation_axis_angle(Vec3_cu& axis) const
    {
        axis.x = h - f + 1e-5f;
        axis.y = c - g;
        axis.z = d - b;
        float sin_angle = axis.safe_normalize();
        float cos_angle = a + e + i - 1.f;
        return atan2(sin_angle, cos_angle);
    }

    IF_CUDA_DEVICE_HOST
    inline Vec3_cu x() const { return Vec3_cu(a, d, g); }
    IF_CUDA_DEVICE_HOST
    inline Vec3_cu y() const { return Vec3_cu(b, e, h); }
    IF_CUDA_DEVICE_HOST
    inline Vec3_cu z() const { return Vec3_cu(c, f, i); }

    // =========================================================================
    /// @name Static constructors
    // =========================================================================

    IF_CUDA_DEVICE_HOST
    static inline Mat3_cu identity()
    {
        return Mat3_cu(1.f, 0.f, 0.f,
                       0.f, 1.f, 0.f,
                       0.f, 0.f, 1.f);

    }

    /// @return the rotation matrix given 'axis' and 'angle' in radian
    IF_CUDA_DEVICE_HOST
    static inline Mat3_cu rotate(const Vec3_cu& axis, float angle)
    {
        Vec3_cu n = axis;
        n.normalize();
        float cp = cosf(angle);
        float sp = sinf(angle);
        float acp = 1.f - cp;
        float nxz = n.x * n.z;
        float nxy = n.x * n.y;
        float nyz = n.y * n.z;
        return Mat3_cu(cp + acp * n.x * n.x,
                       acp * nxy - sp * n.z,
                       acp * nxz + sp * n.y,

                       acp * nxy + sp * n.z,
                       cp + acp * n.y * n.y,
                       acp * nyz - sp * n.x,

                       acp * nxz - sp * n.y,
                       acp * nyz + sp * n.x,
                       cp + acp * n.z * n.z);
    }

    /// @return a orthogonal/normalized frame with its x axis aligned to v
    IF_CUDA_DEVICE_HOST
    static inline Mat3_cu coordinate_system(const Vec3_cu& v)
    {
        Vec3_cu fx, fy, fz;
        fx = v.normalized();
        fx.coordinate_system(fy, fz);
        return Mat3_cu(fx, fy, fz);
    }


};
// =============================================================================

#endif // MAT3_CU_HPP__
