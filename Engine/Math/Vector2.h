#pragma once

#include <cmath>

namespace Engine
{
    struct Vector2
    {
        float X = 0.0f;
        float Y = 0.0f;

        Vector2() = default;

        Vector2(float x, float y)
            : X(x), Y(y)
        {
        }

        Vector2 operator+(const Vector2& other) const
        {
            return 
            {
                X + other.X,
                Y + other.Y
            };
        }

        Vector2 operator-(const Vector2& other) const
        {
            return 
            {
                X - other.X,
                Y - other.Y
            };
        }

        Vector2 operator*(float scalar) const
        {
            return 
            {
                X * scalar,
                Y * scalar
            };
        }

        Vector2 operator/(float scalar) const
        {
            return 
            {
                X / scalar,
                Y / scalar
            };
        }

        Vector2& operator+=(const Vector2& other)
        {
            X += other.X;
            Y += other.Y;

            return *this;
        }

        Vector2& operator-=(const Vector2& other)
        {
            X -= other.X;
            Y -= other.Y;

            return *this;
        }

        Vector2& operator*=(float scalar)
        {
            X *= scalar;
            Y *= scalar;

            return *this;
        }

        float LengthSqured() const
        {
            return X * X + Y * Y;
        }

        float Length() const
        {
            return std::sqrt(LengthSqured());
        }

        Vector2 Normalized() const
        {
            const float length = Length();

            if (length == 0.0f)
            {
                return {0.0f, 0.0f};
            }

            return 
            {
                X / length,
                Y / length
            };
        }

        static float Dot(const Vector2& a, const Vector2& b)
        {
            return 
                a.X * b.X +
                a.Y * b.Y;
        }
    };
}