#pragma once

#include "../Engine/Math/Vector2.h"

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

namespace Tests
{
    struct TestCase
    {
        std::string Name;

        std::function<void()> Function;
    };

    struct TestResult
    {
        std::size_t Passed = 0;

        std::size_t Failed = 0;
    };

    class TestRegistry
    {
    public:

        static TestRegistry& Get();

        void Register(const std::string& name, std::function<void()> function);

        TestResult RunAll();
    private:

        std::vector<TestCase> m_Tests;
    };

    class TestFailure
    {
    public:

        explicit TestFailure(std::string message);

        const std::string& GetMessage() const;

    private:

        std::string m_Message;
    };

    void AssertTrue(bool condition, const char* expression, const char* file, int line);

    void AssertNear(float actual, float expected, float tolerance, const char* actualExpression, const char* expectedExpression, const char* file, int line);

    void AssertVectorNear(const Engine::Vector2& actual, const Engine::Vector2& expected, float tolerance, const char* actualExpression, const char* expectedExpression, const char* file, int line);
}

#define TEST_ASSERT(condition) \
    Tests::AssertTrue( \
        (condition), \
        #condition, \
        __FILE__, \
        __LINE__ \
    )

#define TEST_ASSERT_NEAR(actual, expected, tolerance) \
    Tests::AssertNear( \
        (actual), \
        (expected), \
        (tolerance), \
        #actual, \
        #expected, \
        __FILE__, \
        __LINE__ \
    )

#define TEST_CASE(name) \
    static void name(); \
    \
    namespace \
    { \
        struct name##_Registration \
        { \
            name##_Registration() \
            { \
                Tests::TestRegistry::Get().Register( \
                    #name, \
                    name \
                ); \
            } \
        }; \
        static name##_Registration \
            name##_RegistrationInstance; \
    } \
    \
    static void name()

#define TEST_ASSERT_VECTOR_NEAR(actual, expected, tolerance) \
    Tests::AssertVectorNear( \
        (actual), \
        (expected), \
        (tolerance), \
        #actual, \
        #expected, \
        __FILE__, \
        __LINE__ \
    )