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
        std::cout << "Primitive2DFactory: Creating primitive object...\n";

        Texture2D* texture = PrimitiveTextureFactory2D::GetTexture(shape);

        if (!texture)
        {
            std::cout << "Primitive2DFactory: texture is null\n";

            return nullptr;
        }

        Entity* entity = scene.CreateEntity(GetDefaultName(shape));

        if (!entity)
        {
            std::cout << "Primitive2DFactory: entity is null\n";

            return nullptr;
        }

        TransformComponent* transform = entity->AddComponent<TransformComponent>();

        SpriteRendererComponent* sprite = entity->AddComponent<SpriteRendererComponent>();

        if (!transform || !sprite)
        {
            return entity;
        }

        transform->SetLocalPosition(position);

        sprite->SetTexture(texture);

        const Vector2 textureSize{
            static_cast<float>(texture->GetWidth()),
            static_cast<float>(texture->GetHeight())
        };

        transform->SetLocalScale(
            {
                size.X / textureSize.X,
                size.Y / textureSize.Y
            }
        );

        std::cout << "Primitive Object created\n";

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