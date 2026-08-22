#pragma once

#include "SpriteRegion.h"

namespace Engine
{
    class Texture2D;

    struct SpriteFrame2D
    {
        Texture2D* Texture = nullptr;

        SpriteRegion Region;

        SpriteFrame2D() = default;

        SpriteFrame2D(Texture2D* texture, const SpriteRegion& region)
            : Texture(texture), Region(region)
        {
        }

        bool IsValid() const
        {
            return Texture != nullptr &&
                   Region.GetSize().X > 0.0f &&
                   Region.GetSize().Y > 0.0f;
        }
    };
}