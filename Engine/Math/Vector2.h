#pragma once

#include <cmath>
#include <cstdlib>

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

        Vector2 operator*(const Vector2& other) const
        {
            return 
            {
                X * other.X,
                Y * other.Y
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

        Vector2& operator*=(const Vector2& other)
        {
            X *= other.X;
            Y *= other.Y;

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

        static Vector2 Rotate(const Vector2& vector, float degrees)
        {
            const float radians = degrees * 3.14159265358979323846f / 180.0f;

            const float cosine = std::cos(radians);

            const float sine = std::sin(radians);

            return 
            {
                vector.X * cosine - vector.Y * sine,
                vector.X * sine + vector.Y * cosine
            };
        }

        static Vector2 InverseRotate(const Vector2& vector, float degrees)
        {
            return Rotate(vector, -degrees);
        }

        static Vector2 SafeDivide(const Vector2& value, const Vector2& divisor, float epsilon = 0.000001f)
        {
            Vector2 result = value;

            if (std::abs(divisor.X) > epsilon)
            {
                result.X /= divisor.X;
            }
            else 
            {
                result.X = 0.0f;
            }

            if (std::abs(divisor.Y) > epsilon)
            {
                result.Y /= divisor.Y;
            }
            else 
            {
                result.Y = 0.0f;
            }

            return result;
        }
    };
}