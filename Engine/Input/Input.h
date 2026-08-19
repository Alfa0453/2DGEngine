#pragma once

#include "KeyCode.h"

#include <array>
#include <cstddef>

namespace Engine
{
    class Input
    {
    public:
        void Initialize();

        void Update();

        bool IsKeyDown(KeyCode key) const;

        bool IsKeyUp(KeyCode key) const;

        bool WasKeyPressed(KeyCode key) const;

        bool WasKeyReleased(KeyCode key) const;

    private:
        static constexpr std::size_t KeyboardStateSize = 512;

        std::array<bool, KeyboardStateSize> m_CurrentKeyboardState {};

        std::array<bool, KeyboardStateSize> m_PreviousKeyboardState {};

        int ToSDLScancode(KeyCode key) const;
    };
}