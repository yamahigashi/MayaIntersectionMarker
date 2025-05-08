#ifndef RAY_CU_HPP__
#define RAY_CU_HPP__

#include "vec3_cu.hpp"
#include "point_cu.hpp"
#include "cuda_compiler_interop.hpp"

struct Ray_cu {
    Point_cu _pos;
    Vec3_cu  _dir;

    IF_CUDA_DEVICE_HOST inline
    Ray_cu() : _pos( Point_cu(0.f, 0.f, 0.f) ), _dir( Vec3_cu::zero() ) {  }

    IF_CUDA_DEVICE_HOST inline
    Ray_cu(const Point_cu &pos_, const Vec3_cu &dir_) :
        _pos( pos_ ), _dir( dir_ )
    { }

    /// to set the position
    IF_CUDA_DEVICE_HOST
    inline void set_pos(const Point_cu &pos_) { _pos = pos_; }

    /// to set the direction
    IF_CUDA_DEVICE_HOST
    inline void set_dir(const Vec3_cu &dir_) { _dir = dir_; }

    /// to get the position and the direction for a given axis
    IF_CUDA_DEVICE_HOST
    inline float2 get_dir_pos(int axis_) const {
        float2 ret;
        switch(axis_) {
            case 0:     ret.x = _pos.x; ret.y = _dir.x; return ret;
            case 1:     ret.x = _pos.y; ret.y = _dir.y; return ret;
            default:    ret.x = _pos.z; ret.y = _dir.z; return ret;
        }
    }

    /// get the point at coordinate t along the ray
    IF_CUDA_DEVICE_HOST
    inline Point_cu operator ()(const float t_) const {
        return _pos + _dir * t_;
    }
};


#endif // RAY_CU_HPP__
