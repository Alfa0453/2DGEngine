#pragma once

#include "AnimatorConditionMode.h"

#include <string>

namespace Engine
{
    struct AnimatorCondition2D
    {
        std::string Parameter;

        AnimatorConditionMode Mode = AnimatorConditionMode::IsTrue;

        bool BoolValue = false;

        int IntValue = 0;

        float FloatValue = 0.0f;

        static AnimatorCondition2D Bool(const std::string& parameter, bool expected)
        {
            AnimatorCondition2D condition;

            condition.Parameter = parameter;

            condition.Mode = expected ? AnimatorConditionMode::IsTrue : AnimatorConditionMode::IsFalse;

            condition.BoolValue = expected;

            return condition;
        }

        static AnimatorCondition2D FloatGreater(const std::string& parameter, float value)
        {
            AnimatorCondition2D condition;

            condition.Parameter = parameter;

            condition.Mode = AnimatorConditionMode::Greater;

            condition.FloatValue = value;

            return condition;
        }

        static AnimatorCondition2D FloatLessOrEqual(const std::string& parameter, float value)
        {
            AnimatorCondition2D condition;

            condition.Parameter = parameter;

            condition.Mode = AnimatorConditionMode::LessOrEqual;

            condition.FloatValue = value;

            return condition;
        }

        static AnimatorCondition2D Trigger(const std::string& parameter)
        {
            AnimatorCondition2D condition;

            condition.Parameter = parameter;

            condition.Mode = AnimatorConditionMode::Triggered;

            return condition;
        }
    };
}