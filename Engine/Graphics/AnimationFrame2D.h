#pragma once

#include "SpriteFrame2D.h"


namespace Engine
{
    struct AnimationFrame2D
    {
        SpriteFrame2D SpriteFrame;

        AnimationFrame2D() = default;

        explicit AnimationFrame2D(const SpriteFrame2D& spriteFrame)
            : SpriteFrame(spriteFrame)
        {
        }

        AnimationFrame2D(Texture2D* texture, const SpriteRegion& region)
            : SpriteFrame(texture, region)
        {
        }
    };
}