#include "Application.h"

#include <iostream>

int main(int argc, char* argv[])
{
    Engine::Application application;

    if (!application.Initialize())
    {
        std::cerr << "Failed to initialize 2DGEngine.\n";

        return 1;
    }

    application.Run();

    application.Shutdown();

    return 0;
}