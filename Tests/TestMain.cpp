#include "TestFramework.h"

int main()
{
    const Tests::TestResult result = Tests::TestRegistry::Get().RunAll();

    return result.Failed == 0 ? 0 : 1;
}