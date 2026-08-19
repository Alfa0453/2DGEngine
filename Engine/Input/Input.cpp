#include "Input.h"

#include <SDL3/SDL.h>

namespace Engine
{
    void Input::Initialize()
    {
        m_CurrentKeyboardState.fill(false);
        m_PreviousKeyboardState.fill(false);

        Update();

        m_PreviousKeyboardState = m_CurrentKeyboardState;
    }

    void Input::Update()
    {
        m_PreviousKeyboardState = m_CurrentKeyboardState;

        int keyCount = 0;

        const bool* keyboardState = SDL_GetKeyboardState(&keyCount);

        if (!keyboardState)
        {
            m_CurrentKeyboardState.fill(false);

            return;
        }

        m_CurrentKeyboardState.fill(false);

        const int count = keyCount < static_cast<int>(KeyboardStateSize)
                                        ? keyCount
                                        : static_cast<int>(KeyboardStateSize);

        for (int i = 0; i < count; ++i)
        {
            m_CurrentKeyboardState[static_cast<std::size_t>(i)] = keyboardState[i];
        }
    }

    bool Input::IsKeyDown(KeyCode key) const
    {
        const int scancode = ToSDLScancode(key);

        if (scancode < 0 || scancode >= static_cast<int>(KeyboardStateSize))
        {
            return false;
        }

        return m_CurrentKeyboardState[static_cast<std::size_t>(scancode)];
    }

    bool Input::IsKeyUp(KeyCode key) const
    {
        return !IsKeyDown(key);
    }

    bool Input::WasKeyPressed(KeyCode key) const
    {
        const int scancode = ToSDLScancode(key);

        if (scancode < 0 || scancode >= static_cast<int>(KeyboardStateSize))
        {
            return false;
        }

        const std::size_t index = static_cast<std::size_t>(scancode);

        return m_CurrentKeyboardState[index] && !m_PreviousKeyboardState[index];
    }

    bool Input::WasKeyReleased(KeyCode key) const
    {
        const int scancode = ToSDLScancode(key);

        if (scancode < 0 || scancode >= static_cast<int>(KeyboardStateSize))
        {
            return false;
        }

        const std::size_t index = static_cast<std::size_t>(scancode);

        return !m_CurrentKeyboardState[index] && m_PreviousKeyboardState[index];
    }

    int Input::ToSDLScancode(KeyCode key) const
    {
        switch (key)
        {
            case KeyCode::A:
                return SDL_SCANCODE_A;

            case KeyCode::B:
                return SDL_SCANCODE_B;

            case KeyCode::C:
                return SDL_SCANCODE_C;

            case KeyCode::D:
                return SDL_SCANCODE_D;

            case KeyCode::E:
                return SDL_SCANCODE_E;

            case KeyCode::F:
                return SDL_SCANCODE_F;

            case KeyCode::G:
                return SDL_SCANCODE_G;

            case KeyCode::H:
                return SDL_SCANCODE_H;

            case KeyCode::I:
                return SDL_SCANCODE_I;

            case KeyCode::J:
                return SDL_SCANCODE_J;

            case KeyCode::K:
                return SDL_SCANCODE_K;

            case KeyCode::L:
                return SDL_SCANCODE_L;

            case KeyCode::M:
                return SDL_SCANCODE_M;

            case KeyCode::N:
                return SDL_SCANCODE_N;

            case KeyCode::O:
                return SDL_SCANCODE_O;

            case KeyCode::P:
                return SDL_SCANCODE_P;

            case KeyCode::Q:
                return SDL_SCANCODE_Q;

            case KeyCode::R:
                return SDL_SCANCODE_R;

            case KeyCode::S:
                return SDL_SCANCODE_S;

            case KeyCode::T:
                return SDL_SCANCODE_T;

            case KeyCode::U:
                return SDL_SCANCODE_U;

            case KeyCode::V:
                return SDL_SCANCODE_V;

            case KeyCode::W:
                return SDL_SCANCODE_W;

            case KeyCode::X:
                return SDL_SCANCODE_X;

            case KeyCode::Y:
                return SDL_SCANCODE_Y;

            case KeyCode::Z:
                return SDL_SCANCODE_Z;

            case KeyCode::Num0:
                return SDL_SCANCODE_0;

            case KeyCode::Num1:
                return SDL_SCANCODE_1;

            case KeyCode::Num2:
                return SDL_SCANCODE_2;

            case KeyCode::Num3:
                return SDL_SCANCODE_3;

            case KeyCode::Num4:
                return SDL_SCANCODE_4;

            case KeyCode::Num5:
                return SDL_SCANCODE_5;

            case KeyCode::Num6:
                return SDL_SCANCODE_6;

            case KeyCode::Num7:
                return SDL_SCANCODE_7;

            case KeyCode::Num8:
                return SDL_SCANCODE_8;

            case KeyCode::Num9:
                return SDL_SCANCODE_9;

            case KeyCode::Escape:
                return SDL_SCANCODE_ESCAPE;

            case KeyCode::Space:
                return SDL_SCANCODE_SPACE;

            case KeyCode::Enter:
                return SDL_SCANCODE_RETURN;

            case KeyCode::Tab:
                return SDL_SCANCODE_TAB;

            case KeyCode::Backspace:
                return SDL_SCANCODE_BACKSPACE;

            case KeyCode::Left:
                return SDL_SCANCODE_LEFT;

            case KeyCode::Right:
                return SDL_SCANCODE_RIGHT;

            case KeyCode::Up:
                return SDL_SCANCODE_UP;

            case KeyCode::Down:
                return SDL_SCANCODE_DOWN;

            case KeyCode::LeftShift:
                return SDL_SCANCODE_LSHIFT;

            case KeyCode::RightShift:
                return SDL_SCANCODE_RSHIFT;

            case KeyCode::LeftControl:
                return SDL_SCANCODE_LCTRL;

            case KeyCode::RightControl:
                return SDL_SCANCODE_RCTRL;

            case KeyCode::LeftAlt:
                return SDL_SCANCODE_LALT;

            case KeyCode::RightAlt:
                return SDL_SCANCODE_RALT;

            default:
                return -1;
        }
    }
}