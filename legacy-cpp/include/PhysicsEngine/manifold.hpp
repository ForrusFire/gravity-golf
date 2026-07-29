#ifndef MANIFOLD_HPP
#define MANIFOLD_HPP


#include "PhysicsEngine/object.hpp"


// Manifold
struct Manifold
{
    Object *A;
    Object *B;
    float penetration;
    sf::Vector2f normal;
};


#endif