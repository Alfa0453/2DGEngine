#pragma once

#include <cstdint>

namespace Engine
{
    struct Color
    {
        std::uint8_t R = 255;
        std::uint8_t G = 255;
        std::uint8_t B = 255;
        std::uint8_t A = 255;

        Color() = default;

        Color(std::uint8_t r, std::uint8_t g, std::uint8_t b, std::uint8_t a = 255)
            : R(r), G(g), B(b), A(a)
        {
        }

        static Color White()
        {
            return {255, 255, 255, 255};
        }

        static Color Black()
        {
            return {0, 0, 0, 255};
        }
        static Color Red()
        {
            return {255, 0, 0, 255};
        }

        static Color Green()
        {
            return {0, 255, 0, 255};
        }

        static Color Blue()
        {
            return {0, 0, 255, 255};
        }

        static Color Transparent()
        {
            return {0, 0, 0, 0};
        }
    };
}