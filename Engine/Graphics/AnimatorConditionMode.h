#pragma once

namespace Engine
{
    enum class AnimatorConditionMode
    {
        IsTrue,
        IsFalse,

        Equals,
        NotEquals,

        Greater,
        GreaterOrEqual,

        Less,
        LessOrEqual,

        Triggered
    };
}