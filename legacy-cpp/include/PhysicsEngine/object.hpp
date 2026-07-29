#ifndef OBJECT_HPP
#define OBJECT_HPP


#include "PhysicsEngine/math.hpp"
#include "PhysicsEngine/shape.hpp"


struct Object
{
    // Linear Motion
    sf::Vector2f position;
    sf::Vector2f velocity;
    sf::Vector2f force;

    // Rotational Motion
    float theta;
    float angular_velocity;
    float torque;

    // Object Shape and Material
    Shape *shape;
    Material material;

    // Object Properties
    MassData mass_data;
    float gravity_scale;
};


struct Material
{
    float density;
    float restitution;
    float static_friction;
    float kinetic_friction;
};


struct MassData
{
    float inv_mass;
    float inv_inertia;
};


#endif