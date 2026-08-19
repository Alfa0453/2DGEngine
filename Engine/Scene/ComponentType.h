#pragma once

#include "ComponentTypeID.h"

namespace Engine
{
    inline ComponentTypeID GetNextComponentTypeID()
    {
        static ComponentTypeID nextTypeID = 1;

        return nextTypeID++;
    }

    template<typename T>
    ComponentTypeID GetComponentTypeID()
    {
        static const ComponentTypeID typeID = GetNextComponentTypeID();

        return typeID;
    }
}