#ifndef Transfo_HPP__
#define Transfo_HPP__

#include "vec3_cu.hpp"
#include "mat3_cu.hpp"
#include "point_cu.hpp"

/**
  @name Transfo
  @brief Handling geometric transformations with a 4x4 matrix


  @see Mat3_cu Vec3_cu
*/
struct Transfo {

    /// Linear matrix storage with <b> rows first (i.e row major) </b>
    /// Using this with OpenGL can be done by transposing first:
    /// @code
    ///     Transfo tr;
    ///     // OpenGL is column major !
    ///     glMultMatrixf( (GLfloat)(tr.transpose().m) );
    /// @endcode
    float m[16];

    // -------------------------------------------------------------------------
    /// @name Constructors
    // -------------------------------------------------------------------------

    IF_CUDA_DEVICE_HOST
    inline Transfo() {}

    IF_CUDA_DEVICE_HOST
    inline Transfo(float a00, float a01, float a02, float a03,
                   float a10, float a11, float a12, float a13,
                   float a20, float a21, float a22, float a23,
                   float a30, float a31, float a32, float a33)
    {
        m[ 0] = a00; m[ 1] = a01; m[ 2] = a02; m[ 3] = a03;
        m[ 4] = a10; m[ 5] = a11; m[ 6] = a12; m[ 7] = a13;
        m[ 8] = a20; m[ 9] = a21; m[10] = a22; m[11] = a23;
        m[12] = a30; m[13] = a31; m[14] = a32; m[15] = a33;
    }

    IF_CUDA_DEVICE_HOST
    inline Transfo(const Mat3_cu& x){
        m[ 0] = x.a; m[ 1] = x.b; m[ 2] = x.c; m[ 3] = 0.f;
        m[ 4] = x.d; m[ 5] = x.e; m[ 6] = x.f; m[ 7] = 0.f;
        m[ 8] = x.g; m[ 9] = x.h; m[10] = x.i; m[11] = 0.f;
        m[12] = 0.f; m[13] = 0.f; m[14] = 0.f; m[15] = 1.f;
    }

    IF_CUDA_DEVICE_HOST
    inline Transfo(const Mat3_cu& x, const Vec3_cu& v){
        m[ 0] = x.a; m[ 1] = x.b; m[ 2] = x.c; m[ 3] = v.x;
        m[ 4] = x.d; m[ 5] = x.e; m[ 6] = x.f; m[ 7] = v.y;
        m[ 8] = x.g; m[ 9] = x.h; m[10] = x.i; m[11] = v.z;
        m[12] = 0.f; m[13] = 0.f; m[14] = 0.f; m[15] = 1.f;
    }

    IF_CUDA_DEVICE_HOST
    inline Transfo(const Vec3_cu& v){
        m[ 0] = 1.f; m[ 1] = 0.f; m[ 2] = 0.f; m[ 3] = v.x;
        m[ 4] = 0.f; m[ 5] = 1.f; m[ 6] = 0.f; m[ 7] = v.y;
        m[ 8] = 0.f; m[ 9] = 0.f; m[10] = 1.f; m[11] = v.z;
        m[12] = 0.f; m[13] = 0.f; m[14] = 0.f; m[15] = 1.f;
    }

    // -------------------------------------------------------------------------
    /// @name Setters
    // -------------------------------------------------------------------------

    IF_CUDA_DEVICE_HOST
    inline void set_x(const Vec3_cu& x){
        m[0] = x.x; m[4] = x.y; m[8] = x.z;
    }

    IF_CUDA_DEVICE_HOST
    inline void set_y(const Vec3_cu& y){
        m[1] = y.x; m[5] = y.y; m[9] = y.z;
    }

    IF_CUDA_DEVICE_HOST
    inline void set_z(const Vec3_cu& z){
        m[2] = z.x; m[6] = z.y; m[10] = z.z;
    }

    IF_CUDA_DEVICE_HOST
    inline Vec3_cu x() const{
        return Vec3_cu( m[0], m[4], m[8] );
    }

    IF_CUDA_DEVICE_HOST
    inline Vec3_cu y() const{
        return Vec3_cu( m[1], m[5], m[9] );
    }

    IF_CUDA_DEVICE_HOST
    inline Vec3_cu z() const{
        return Vec3_cu( m[2], m[6], m[10] );
    }

    IF_CUDA_DEVICE_HOST
    inline void set_translation(const Vec3_cu& tr){
        m[3] = tr.x; m[7] = tr.y; m[11] = tr.z;
    }

    IF_CUDA_DEVICE_HOST
    inline void set_org(const Vec3_cu& tr){
        m[3] = tr.x; m[7] = tr.y; m[11] = tr.z;
    }

    IF_CUDA_DEVICE_HOST
    inline void set_translation(const Transfo& tr){
        const Vec3_cu trans = tr.get_translation();
        m[3] = trans.x; m[7] = trans.y; m[11] = trans.z;
    }

    IF_CUDA_DEVICE_HOST
    inline void set_mat3(const Mat3_cu& x){
        m[ 0] = x.a; m[ 1] = x.b; m[ 2] = x.c;
        m[ 4] = x.d; m[ 5] = x.e; m[ 6] = x.f;
        m[ 8] = x.g; m[ 9] = x.h; m[10] = x.i;
    }

    // -------------------------------------------------------------------------
    /// @name Operators
    /// @note A special attention has to be made regarding the multiplication
    /// operators. The operators are overloaded differently wether you use
    /// vectors or points. Vectors does not need to be multiplied against the
    /// matrix translation part, whereas points does. This is because the
    /// homogenous part is not represented. Yet when projecting a point you will
    /// need the fourth component. In this case projection can't be done through
    /// matrix multiplication use instead the method 'project()'
    // -------------------------------------------------------------------------

    IF_CUDA_DEVICE_HOST
    inline Vec3_cu operator*(const Vec3_cu& v) const {
        return Vec3_cu(
                    m[0] * v.x + m[1] * v.y + m[ 2] * v.z,
                    m[4] * v.x + m[5] * v.y + m[ 6] * v.z,
                    m[8] * v.x + m[9] * v.y + m[10] * v.z);
    }

    /// @warning this ignores the homogenous coordinates use project() to
    /// consider the fourth component and do the perspective division for
    /// projection matrices
    IF_CUDA_DEVICE_HOST
    inline Point_cu operator*(const Point_cu& v) const {
        return Point_cu(
                    m[0] * v.x + m[1] * v.y + m[ 2] * v.z + m[ 3],
                    m[4] * v.x + m[5] * v.y + m[ 6] * v.z + m[ 7],
                    m[8] * v.x + m[9] * v.y + m[10] * v.z + m[11]);
    }

    // Multiply a vector by a matrix, as if the vector was a point.
    //
    // The only difference is that translations are normally applied only to points
    // and not vectors.  If you have a vector that actually does need translations
    // applied, you can use this instead of converting to a point and back.
    IF_CUDA_DEVICE_HOST
    inline Point_cu multiply_as_point(const Vec3_cu& v) const {
        return Point_cu(
                    m[0] * v.x + m[1] * v.y + m[ 2] * v.z + m[ 3],
                    m[4] * v.x + m[5] * v.y + m[ 6] * v.z + m[ 7],
                    m[8] * v.x + m[9] * v.y + m[10] * v.z + m[11]);
    }

    /// Multiply 'v' by the matrix and do the perspective division
    IF_CUDA_DEVICE_HOST
    inline Point_cu project(const Point_cu& v) const {
        Point_cu tmp =  Point_cu(
                    m[0] * v.x + m[1] * v.y + m[ 2] * v.z + m[ 3],
                    m[4] * v.x + m[5] * v.y + m[ 6] * v.z + m[ 7],
                    m[8] * v.x + m[9] * v.y + m[10] * v.z + m[11]);

        return tmp / (m[12] * v.x + m[13] * v.y + m[14] * v.z + m[15]);
    }


    IF_CUDA_DEVICE_HOST
    inline Transfo operator*(const Transfo& t) const {
        Transfo res;
        for(int i = 0; i < 4; i++){
            int j = i*4;
            res[j+0] = m[j] * t.m[0] + m[j+1] * t.m[4] + m[j+2] * t.m[ 8] + m[j+3] * t.m[12];
            res[j+1] = m[j] * t.m[1] + m[j+1] * t.m[5] + m[j+2] * t.m[ 9] + m[j+3] * t.m[13];
            res[j+2] = m[j] * t.m[2] + m[j+1] * t.m[6] + m[j+2] * t.m[10] + m[j+3] * t.m[14];
            res[j+3] = m[j] * t.m[3] + m[j+1] * t.m[7] + m[j+2] * t.m[11] + m[j+3] * t.m[15];
        }
        return res;
    }

    IF_CUDA_DEVICE_HOST
    inline Transfo& operator*=(const Transfo& t){
        (*this) = (*this) * t;
        return (*this);
    }

    IF_CUDA_DEVICE_HOST
    inline Vec3_cu vec_prod(const Vec3_cu& v) const {
        return Vec3_cu(
                    m[0] * v.x + m[1] * v.y + m[ 2] * v.z,
                    m[4] * v.x + m[5] * v.y + m[ 6] * v.z,
                    m[8] * v.x + m[9] * v.y + m[10] * v.z);
    }

    IF_CUDA_DEVICE_HOST
    inline Transfo operator*(float x) const {
        Transfo res;
        for(int i = 0; i < 16; i++){
            res[i] = m[i] * x;
        }
        return res;
    }

    IF_CUDA_DEVICE_HOST
    inline Transfo operator+(const Transfo& t) const{
        Transfo res;
        for(int i = 0; i < 16; i++){
            res[i] = m[i] + t.m[i];
        }
        return res;
    }

    IF_CUDA_DEVICE_HOST
    inline Transfo& operator+=(const Transfo& t){
        (*this) = (*this) + t;
        return (*this);
    }

    IF_CUDA_DEVICE_HOST
    inline Transfo operator-(const Transfo& t) const{
        Transfo res;
        for(int i = 0; i < 16; i++){
            res[i] = m[i] - t.m[i];
        }
        return res;
    }

    IF_CUDA_DEVICE_HOST
    inline Transfo& operator-=(const Transfo& t){
        (*this) = (*this) - t;
        return (*this);
    }

    IF_CUDA_DEVICE_HOST
    inline float& operator[](int i) {
        return m[i];
    }

    IF_CUDA_DEVICE_HOST
    inline const float& operator[](int i) const {
        return m[i];
    }

    // -------------------------------------------------------------------------
    /// @name Getters
    // -------------------------------------------------------------------------


    IF_CUDA_DEVICE_HOST
    inline Transfo transpose() const {
        return Transfo(m[0], m[4], m[ 8], m[12],
                       m[1], m[5], m[ 9], m[13],
                       m[2], m[6], m[10], m[14],
                       m[3], m[7], m[11], m[15]);
    }

    /// Fast inversion of the Transformation matrix. To accelerate computation
    /// we consider that the matrix only represents affine Transformations
    /// such as rotation, scaling, translation, shear...
    /// @warning don't use this procedure to invert a projection matrix. Instead
    /// use full_invert().
    IF_CUDA_DEVICE_HOST
    inline Transfo fast_invert() const {
        Mat3_cu a(m[0], m[1], m[ 2],
                  m[4], m[5], m[ 6],
                  m[8], m[9], m[10]);

        Vec3_cu b(m[3], m[7], m[11]);
        Mat3_cu x = a.inverse();
        Vec3_cu y = x * b;
        return Transfo(x.a, x.b, x.c, -y.x,
                       x.d, x.e, x.f, -y.y,
                       x.g, x.h, x.i, -y.z,
                       0.f, 0.f, 0.f,  1.f);
    }

    IF_CUDA_DEVICE_HOST static inline
    float MINOR(const Transfo& m, const int r0, const int r1, const int r2, const int c0, const int c1, const int c2) {
        return m[4*r0+c0] * (m[4*r1+c1] * m[4*r2+c2] - m[4*r2+c1] * m[4*r1+c2]) -
               m[4*r0+c1] * (m[4*r1+c0] * m[4*r2+c2] - m[4*r2+c0] * m[4*r1+c2]) +
               m[4*r0+c2] * (m[4*r1+c0] * m[4*r2+c1] - m[4*r2+c0] * m[4*r1+c1]);
    }

    IF_CUDA_DEVICE_HOST inline
    Transfo adjoint() const {
        return Transfo( MINOR(*this,1,2,3,1,2,3), -MINOR(*this,0,2,3,1,2,3),  MINOR(*this,0,1,3,1,2,3), -MINOR(*this,0,1,2,1,2,3),
                       -MINOR(*this,1,2,3,0,2,3),  MINOR(*this,0,2,3,0,2,3), -MINOR(*this,0,1,3,0,2,3),  MINOR(*this,0,1,2,0,2,3),
                        MINOR(*this,1,2,3,0,1,3), -MINOR(*this,0,2,3,0,1,3),  MINOR(*this,0,1,3,0,1,3), -MINOR(*this,0,1,2,0,1,3),
                       -MINOR(*this,1,2,3,0,1,2),  MINOR(*this,0,2,3,0,1,2), -MINOR(*this,0,1,3,0,1,2),  MINOR(*this,0,1,2,0,1,2));
    }

    IF_CUDA_DEVICE_HOST
    inline float det() const {
        return m[0] * MINOR(*this, 1, 2, 3, 1, 2, 3) -
               m[1] * MINOR(*this, 1, 2, 3, 0, 2, 3) +
               m[2] * MINOR(*this, 1, 2, 3, 0, 1, 3) -
               m[3] * MINOR(*this, 1, 2, 3, 0, 1, 2);
    }

    /// Full inversion of the Transformation matrix. No assumption is made about
    /// the 4x4 matrix to optimize inversion. if the Transformation is
    /// not affine you must use this procedure to invert the matrix. For
    /// instance perspective projection can't use the fast_invert() procedure
    /// @see fast_invert()
    IF_CUDA_DEVICE_HOST
    inline Transfo full_invert() const {
        return adjoint() * (1.0f / det());
    }

    /// @return the Transformation with normalized x, y, z column vectors
    IF_CUDA_DEVICE_HOST
    inline Transfo normalized() const {
        return Transfo(get_mat3().normalized(), get_translation());
    }

    IF_CUDA_DEVICE_HOST
    inline Mat3_cu get_mat3() const {
        return Mat3_cu(m[0], m[1], m[2],
                       m[4], m[5], m[6],
                       m[8], m[9], m[10]);
    }

    /// get translation part of the matrix
    /// @note same as get_org()
    IF_CUDA_DEVICE_HOST
    inline Vec3_cu get_translation() const {
        return Vec3_cu(m[3], m[7], m[11]);
    }

    /// get origine of the frame represented by the matrix
    /// @note same as get_translation()
    IF_CUDA_DEVICE_HOST
    inline Vec3_cu get_org() const {
        return Vec3_cu(m[3], m[7], m[11]);
    }

    /// Check if the vectors representing the frame are orthogonals.
    /// @warning Don't mix up this with orthogonal matrices.
    IF_CUDA_DEVICE_HOST
    inline bool is_frame_ortho(float eps = 0.0001f) const {
        return fabsf( x().dot( y() ) ) < eps &&
               fabsf( x().dot( z() ) ) < eps &&
               fabsf( y().dot( z() ) ) < eps;
    }

    // Return true if the transforms are equal, within a margin of error.
    inline bool equal(const Transfo &rhs) {
        for(int i = 0; i < 16; ++i)
            if(abs((*this)[i] - rhs[i]) > 1e-6)
                return false;
        return true;
    }

    inline
    void print() const
    {
        printf("%f %f %f %f\n", m[0 ], m[1 ], m[2 ], m[3 ] );
        printf("%f %f %f %f\n", m[4 ], m[5 ], m[6 ], m[7 ] );
        printf("%f %f %f %f\n", m[8 ], m[9 ], m[10], m[11] );
        printf("%f %f %f %f\n", m[12], m[13], m[14], m[15] );
    }

    // -------------------------------------------------------------------------
    /// @name Static transformation generators
    // -------------------------------------------------------------------------

    IF_CUDA_DEVICE_HOST
    static inline Transfo translate(float dx, float dy, float dz){
        return Transfo(1.f, 0.f, 0.f, dx,
                       0.f, 1.f, 0.f, dy,
                       0.f, 0.f, 1.f, dz,
                       0.f, 0.f, 0.f, 1.f);
    }

    IF_CUDA_DEVICE_HOST
    static inline Transfo translate(const Vec3_cu& v){
        return Transfo::translate(v.x, v.y, v.z);
    }

    IF_CUDA_DEVICE_HOST
    static inline Transfo scale(float sx, float sy, float sz){
        return Transfo( sx,  0.f, 0.f, 0.f,
                        0.f,  sy, 0.f, 0.f,
                        0.f, 0.f,  sz, 0.f,
                        0.f, 0.f, 0.f, 1.f);
    }

    IF_CUDA_DEVICE_HOST
    static inline Transfo scale(const Vec3_cu& v){
        return Transfo::scale(v.x, v.y, v.z);
    }

    /// Build a uniform scaling matrix on x,y,z.
    IF_CUDA_DEVICE_HOST
    static inline Transfo scale(float s){
        return Transfo::scale(s, s, s);
    }

    IF_CUDA_DEVICE_HOST
    static inline Transfo rotate(const Vec3_cu& center,
                                 const Vec3_cu& axis,
                                 float angle,
                                 const Mat3_cu& frame)
    {
        Transfo r(frame * Mat3_cu::rotate(axis, angle) * frame.inverse());
        return Transfo::translate(center) * r * Transfo::translate(-center);
    }

    IF_CUDA_DEVICE_HOST
    static inline Transfo rotate(const Vec3_cu& center,
                                 const Vec3_cu& axis,
                                 float angle)
    {
        Transfo r(Mat3_cu::rotate(axis, angle));
        return Transfo::translate(center) * r * Transfo::translate(-center);
    }

    /// build a rotation matrix around the origin.
    /// @param axis : the <b> normalized </b> axis of rotation
    /// @param angle : rotation's angle in radian
    IF_CUDA_DEVICE_HOST
    static inline Transfo rotate(const Vec3_cu& axis, float angle)
    {
        return Transfo(Mat3_cu::rotate(axis, angle));
    }

    IF_CUDA_DEVICE_HOST
    static inline Transfo identity(){
        return Transfo(1.f, 0.f, 0.f, 0.f,
                       0.f, 1.f, 0.f, 0.f,
                       0.f, 0.f, 1.f, 0.f,
                       0.f, 0.f, 0.f, 1.f);
    }

    IF_CUDA_DEVICE_HOST
    static inline Transfo empty(){
        return Transfo(0.f, 0.f, 0.f, 0.f,
                       0.f, 0.f, 0.f, 0.f,
                       0.f, 0.f, 0.f, 0.f,
                       0.f, 0.f, 0.f, 0.f);
    }
};

#endif // Transfo_HPP__
