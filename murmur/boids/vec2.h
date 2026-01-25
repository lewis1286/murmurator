#pragma once
#ifndef VEC2_H
#define VEC2_H

#include <cmath>

namespace murmur {

struct Vec2 {
    float x;
    float y;

    Vec2() : x(0.0f), y(0.0f) {}
    Vec2(float x_, float y_) : x(x_), y(y_) {}

    Vec2 operator+(const Vec2& other) const {
        return Vec2(x + other.x, y + other.y);
    }

    Vec2 operator-(const Vec2& other) const {
        return Vec2(x - other.x, y - other.y);
    }

    Vec2 operator*(float scalar) const {
        return Vec2(x * scalar, y * scalar);
    }

    Vec2 operator/(float scalar) const {
        if (scalar == 0.0f) return Vec2(0.0f, 0.0f);
        return Vec2(x / scalar, y / scalar);
    }

    Vec2& operator+=(const Vec2& other) {
        x += other.x;
        y += other.y;
        return *this;
    }

    Vec2& operator-=(const Vec2& other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    Vec2& operator*=(float scalar) {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    float Magnitude() const {
        return sqrtf(x * x + y * y);
    }

    float MagnitudeSquared() const {
        return x * x + y * y;
    }

    Vec2 Normalized() const {
        float mag = Magnitude();
        if (mag < 0.0001f) return Vec2(0.0f, 0.0f);
        return Vec2(x / mag, y / mag);
    }

    void Normalize() {
        float mag = Magnitude();
        if (mag > 0.0001f) {
            x /= mag;
            y /= mag;
        }
    }

    void Limit(float max) {
        float mag_sq = MagnitudeSquared();
        if (mag_sq > max * max) {
            float mag = sqrtf(mag_sq);
            x = (x / mag) * max;
            y = (y / mag) * max;
        }
    }

    void SetMagnitude(float mag) {
        Normalize();
        x *= mag;
        y *= mag;
    }

    float Dot(const Vec2& other) const {
        return x * other.x + y * other.y;
    }

    float Angle() const {
        return atan2f(y, x);
    }

    static float Distance(const Vec2& a, const Vec2& b) {
        float dx = a.x - b.x;
        float dy = a.y - b.y;
        return sqrtf(dx * dx + dy * dy);
    }

    static float DistanceSquared(const Vec2& a, const Vec2& b) {
        float dx = a.x - b.x;
        float dy = a.y - b.y;
        return dx * dx + dy * dy;
    }
};

} // namespace murmur

#endif // VEC2_H
