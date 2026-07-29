#include "PhysicsEngine/math.hpp"


// Return the val when bounded by the min and the max
float bound(float min, float max, float val) 
{
    if (val <= min) {
        return min;
    }
    
    if (val >= max) {
        return max;
    }

    return val;
}


// Calculates the dot product between two vectors
float dot_product(sf::Vector2f A, sf::Vector2f B)
{
    return A.x * B.x + A.y * B.y;
}


// Calculates the length squared of a vector
float length_squared(sf::Vector2f A)
{
    return dot_product(A, A);
}


// Calculates the length of a vector. Use length_squared() if possible
// to prevent unnecessary square root calculations
float length(sf::Vector2f A)
{
    return sqrt(length_squared(A));
}


// Normalize a vector
sf::Vector2f normalize(sf::Vector2f A)
{
    float len = length(A);
    return A / len;
}


// Calculates the cross product between two vectors
float cross_product(sf::Vector2f A, sf::Vector2f B)
{
    return A.x * B.y - A.y * B.x;
}

