#pragma once
#ifndef VEC3_H
#define VEC3_H

#include <cmath>

namespace murmur {

struct Vec3 {
    float x;
    float y;
    float z;

    Vec3() : x(0.0f), y(0.0f), z(0.0f) {}
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3& other) const {
        return Vec3(x + other.x, y + other.y, z + other.z);
    }

    Vec3 operator-(const Vec3& other) const {
        return Vec3(x - other.x, y - other.y, z - other.z);
    }

    Vec3 operator*(float scalar) const {
        return Vec3(x * scalar, y * scalar, z * scalar);
    }

    Vec3 operator/(float scalar) const {
        if (scalar == 0.0f) return Vec3(0.0f, 0.0f, 0.0f);
        return Vec3(x / scalar, y / scalar, z / scalar);
    }

    Vec3& operator+=(const Vec3& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    Vec3& operator-=(const Vec3& other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    Vec3& operator*=(float scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    float Magnitude() const {
        return sqrtf(x * x + y * y + z * z);
    }

    float MagnitudeSquared() const {
        return x * x + y * y + z * z;
    }

    Vec3 Normalized() const {
        float mag = Magnitude();
        if (mag < 0.0001f) return Vec3(0.0f, 0.0f, 0.0f);
        return Vec3(x / mag, y / mag, z / mag);
    }

    void Normalize() {
        float mag = Magnitude();
        if (mag > 0.0001f) {
            x /= mag;
            y /= mag;
            z /= mag;
        }
    }

    void Limit(float max) {
        float mag_sq = MagnitudeSquared();
        if (mag_sq > max * max) {
            float mag = sqrtf(mag_sq);
            x = (x / mag) * max;
            y = (y / mag) * max;
            z = (z / mag) * max;
        }
    }

    void SetMagnitude(float mag) {
        Normalize();
        x *= mag;
        y *= mag;
        z *= mag;
    }

    float Dot(const Vec3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    // Angle of x-y projection (for display heading)
    float AngleXY() const {
        return atan2f(y, x);
    }

    static float Distance(const Vec3& a, const Vec3& b) {
        float dx = a.x - b.x;
        float dy = a.y - b.y;
        float dz = a.z - b.z;
        return sqrtf(dx * dx + dy * dy + dz * dz);
    }

    static float DistanceSquared(const Vec3& a, const Vec3& b) {
        float dx = a.x - b.x;
        float dy = a.y - b.y;
        float dz = a.z - b.z;
        return dx * dx + dy * dy + dz * dz;
    }
};

} // namespace murmur

#endif // VEC3_H
