#include "kinematics/types.h"
#include <cmath>

namespace vmc {

float Vec3::norm() const {
    return std::sqrt(x*x + y*y + z*z);
}

Vec3 Vec3::normalized() const {
    float n = norm();
    if (n < 1e-9f) return {0, 0, 0};
    return {x/n, y/n, z/n};
}

float dot(const Vec3& a, const Vec3& b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x};
}

Mat3 Mat3::zero() {
    Mat3 r;
    for (int i = 0; i < 9; ++i) r.m[i] = 0;
    return r;
}

Mat3 Mat3::identity() {
    Mat3 r;
    r.m[0] = 1; r.m[4] = 1; r.m[8] = 1;
    return r;
}

// 绕 x 轴的旋转矩阵
Mat3 Mat3::rotationX(float a) {
    float c = std::cos(a), s = std::sin(a);
    Mat3 r;
    r.m[0] = 1;
    r.m[4] = c;  r.m[5] = -s;
    r.m[7] = s;  r.m[8] = c;
    return r;
}

// 绕 y 轴的旋转矩阵
Mat3 Mat3::rotationY(float a) {
    float c = std::cos(a), s = std::sin(a);
    Mat3 r;
    r.m[0] = c;  r.m[2] = s;
    r.m[4] = 1;
    r.m[6] = -s; r.m[8] = c;
    return r;
}

// 绕 z 轴的旋转矩阵
Mat3 Mat3::rotationZ(float a) {
    float c = std::cos(a), s = std::sin(a);
    Mat3 r;
    r.m[0] = c;  r.m[1] = -s;
    r.m[3] = s;  r.m[4] = c;
    r.m[8] = 1;
    return r;
}

Vec3 Mat3::operator*(const Vec3& v) const {
    return {
        m[0]*v.x + m[1]*v.y + m[2]*v.z,
        m[3]*v.x + m[4]*v.y + m[5]*v.z,
        m[6]*v.x + m[7]*v.y + m[8]*v.z
    };
}

Mat3 Mat3::operator*(const Mat3& o) const {
    Mat3 r;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            r.m[i*3 + j] = m[i*3]*o.m[j] + m[i*3+1]*o.m[3+j] + m[i*3+2]*o.m[6+j];
        }
    }
    return r;
}

Mat3 Mat3::transpose() const {
    Mat3 r;
    r.m[0] = m[0]; r.m[1] = m[3]; r.m[2] = m[6];
    r.m[3] = m[1]; r.m[4] = m[4]; r.m[5] = m[7];
    r.m[6] = m[2]; r.m[7] = m[5]; r.m[8] = m[8];
    return r;
}

} // namespace vmc
