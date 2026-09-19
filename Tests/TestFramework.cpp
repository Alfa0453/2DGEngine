#include "TestFramework.h"

#include <iostream>
#include <sstream>
#include <utility>
#include <cmath>

namespace Tests
{
    TestRegistry& TestRegistry::Get()
    {
        static TestRegistry registry;

        return registry;
    }

    void TestRegistry::Register(const std::string& name, std::function<void()> function)
    {
        TestCase test;

        test.Name = name;

        test.Function = std::move(function);

        m_Tests.push_back(std::move(test));
    }

    TestFailure::TestFailure(std::string message)
        : m_Message(std::move(message))
    {
    }

    const std::string& TestFailure::GetMessage() const
    {
        return m_Message;
    }

    void AssertTrue(bool condition, const char *expression, const char *file, int line)
    {
        if (condition)
        {
            return;
        }

        std::ostringstream stream;

        stream 
            << file
            << ':'
            << line
            << " Assertion failed: "
            << expression;

        throw TestFailure(stream.str());
    }

    void AssertNear(float actual, float expected, float tolerance, const char *actualExpression, const char *expectedExpression, const char *file, int line)
    {
        const float difference = actual > expected ? actual - expected : expected - actual;

        if (difference <= tolerance)
        {
            return;
        }

        std::ostringstream stream;

        stream
            << file
            << ':'
            << line
            << " Expected "
            << actualExpression
            << " ~= "
            << expectedExpression
            << ", actual="
            << actual
            << ", expected="
            << expected
            << ", tolerance="
            << tolerance;

        throw TestFailure(stream.str());
    }

    void AssertVectorNear(const Engine::Vector2& actual, const Engine::Vector2& expected, float tolerance, const char* actualExpression, const char* expectedExpression, const char* file, int line)
    {
        const float differenceX = std::abs(actual.X - expected.X);

        const float differenceY = std::abs(actual.Y - expected.Y);

        if (differenceX <= tolerance && differenceY <= tolerance)
        {
            return;
        }

        std::ostringstream stream;

        stream
            << file << ':' << line << " Expected "
            << actualExpression << " ~= " 
            << expectedExpression << '\n'
            << " actual=(" << actual.X << ", "
            << actual.Y << ") expected=("
            << expected.X << ", " << expected.Y << ')';

            throw TestFailure(stream.str());
    }

    TestResult TestRegistry::RunAll()
    {
        TestResult result;

        std::cout << "\n=== 2DGEngine Tests ===\n\n";

        for (const TestCase& test : m_Tests)
        {
            try
            {
                test.Function();

                ++result.Passed;

                std::cout << "[PASS] " << test.Name << '\n';
            }
            catch (const TestFailure& failure)
            {
                ++result.Failed;

                std::cout << "[FAIL] " << test.Name << '\n'
                          << "       "
                          << failure.GetMessage() << '\n';
            }
            catch (...)
            {
                ++result.Failed;

                std::cout << "[FAIL] " << test.Name << '\n'
                          << "       Unexpected exception\n";
            }
        }

        std::cout 
            << "\n===========================\n"
            << "Passed: " << result.Passed << '\n'
            << "Failed: " << result.Failed << '\n'
            << "Total: " << result.Passed + result.Failed << '\n'
            << "==============================\n";

        return result;
    }
}