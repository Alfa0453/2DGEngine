#pragma once

#include "Vector2.h"

#include <cmath>
#include <cstdlib>

namespace Engine
{
    struct Matrix2x2
    {
        float M00 = 1.0f;
        float M01 = 0.0f;

        float M10 = 0.0f;
        float M11 = 1.0f;

        
        Matrix2x2() = default;

        Matrix2x2(float m00, float m01, float m10, float m11)
            : M00(m00), M01(m01), M10(m10), M11(m11)
        {
        }

        Vector2 Multiply(const Vector2& vector) const
        {
            return 
            {
                M00 * vector.X +
                M01 * vector.Y,

                M10 * vector.X +
                M11 * vector.Y
            };
        }

        bool Inverse(Matrix2x2& outInverse) const
        {
            const float determinant = M00 * M11 - M01 * M10;

            constexpr float epsilon = 0.000001f;

            if (std::abs(determinant) <= epsilon)
            {
                outInverse = Matrix2x2{};

                return false;
            }

            const float inverseDeterminant = 1.0f / determinant;

            outInverse =
            {
                 M11 * inverseDeterminant,
                -M01 * inverseDeterminant,

                -M10 * inverseDeterminant,
                 M00 * inverseDeterminant
            };

            return true;
        }
    };
}