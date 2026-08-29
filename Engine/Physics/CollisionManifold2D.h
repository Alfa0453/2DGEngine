#pragma once

#include "../Math/Vector2.h"

#include <cstddef>

namespace Engine
{
    class Collider2D;

    struct CollisionManifold2D
    {
        Collider2D* A = nullptr;

        Collider2D* B = nullptr;

        // Points from A toward B
        Vector2 Normal{0.0f, 0.0f};

        float Penetration = 0.0f;

        static constexpr std::size_t MaxContacPoints = 2;

        Vector2 ContactPoints[MaxContacPoints]
        {
            Vector2{0.0f, 0.0f},
            Vector2{0.0f, 0.0f}
        };

        float AccumulatedNormalImpulses[MaxContacPoints]{0.0f, 0.0f};

        float AccumulatedTangentImpulses[MaxContacPoints]{0.0f, 0.0f};

        float RestitutionBiases[MaxContacPoints]{0.0f, 0.0f};

        std::size_t ContactCount = 0;

        bool IsTrigger = false;

        bool IsValid() const
        {
            return A != nullptr &&
                   B != nullptr &&
                   Penetration > 0.0f;
        }

        void ClearContacts()
        {
            ContactCount = 0;

            for (std::size_t i = 0; i < MaxContacPoints; ++i)
            {
                ContactPoints[i] = {0.0f, 0.0f};

                AccumulatedNormalImpulses[i] = 0.0f;

                AccumulatedTangentImpulses[i] = 0.0f;

                RestitutionBiases[i] = 0.0f;
            }
        }

        bool AddContactPoint(const Vector2& point)
        {
            if (ContactCount >= MaxContacPoints)
            {
                return false;
            }

            ContactPoints[ContactCount] = point;

            AccumulatedNormalImpulses[ContactCount] = 0.0f;

            AccumulatedTangentImpulses[ContactCount] = 0.0f;

            ++ContactCount;

            return true;
        }
    };
}