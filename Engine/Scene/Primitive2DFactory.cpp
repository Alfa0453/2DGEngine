#include "Primitive2DFactory.h"

#include "../Graphics/Texture2D.h"
#include "../Graphics/PrimitiveTextureFactory2D.h"

#include "Scene.h"
#include "SpriteRendererComponent.h"
#include "TransformComponent.h"

#include <iostream>

namespace Engine
{
    Entity* Primitive2DFactory::Create(Scene &scene, PrimitiveShape2D shape, const Vector2 &position, const Vector2 &size)
    {
        Texture2D* texture = PrimitiveTextureFactory2D::GetTexture(shape);

        if (!texture)
        {
            return nullptr;
        }

        Entity* entity = scene.CreateEntity(GetDefaultName(shape));

        if (!entity)
        {
            return nullptr;
        }

        TransformComponent* transform = entity->AddComponent<TransformComponent>();

        SpriteRendererComponent* sprite = entity->AddComponent<SpriteRendererComponent>();

        if (!transform || !sprite)
        {
            return entity;
        }

        transform->SetLocalPosition(position);

        transform->SetLocalScale({1.0f, 1.0f});

        sprite->SetTexture(texture);

        sprite->SetSize(size);

        return entity;
    }

    const char* Primitive2DFactory::GetDefaultName(PrimitiveShape2D shape)
    {
        switch (shape)
        {
            case PrimitiveShape2D::Square:
                return "Square";

            case PrimitiveShape2D::Rectangle:
                return "Rectangle";

            case PrimitiveShape2D::Circle:
                return "Circle";

            case PrimitiveShape2D::Triangle:
                return "Triangle";
        }

        return "Primitive";
    }
}