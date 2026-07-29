#include "PhysicsEngine/collision.hpp"


/* Detects a circle vs circle collision and generates the manifold if the collision is true */
bool CircleCircleCollision(Manifold *m)
{
    // Set up a couple pointers to each object
    Object *A = m->A;
    Object *B = m->B;
    
    // Vector from A to B
    sf::Vector2f n = B->position - A->position;
    
    float r = A->radius + B->radius;
    float r_squared = r * r;
    
    if (length_squared(n) > r_squared) {
        return false;
    }

    // Circles have collided, now compute manifold
    float d = length(n); // Perform actual sqrt
    
    // If distance between circles is not zero
    if (d != 0)
    {
        // Distance is difference between radius and distance
        m->penetration = r - d;
    
        // Utilize our d since we performed sqrt on it already
        // Points from A to B, and is a unit vector
        m->normal = n / d;
        return true;
    }
    
    // Circles are on same position
    else
    {
        // Choose random (but consistent) values
        m->penetration = A->radius;
        m->normal = sf::Vector2f(1, 0);
        return true;
    }
}


/* Detects an AABB vs AABB collision and generates the manifold if the collision is true */
bool AABBAABBCollision(Manifold *m)
{
    // Setup a couple pointers to each object
    Object *A = m->A;
    Object *B = m->B;
    
    // Vector from A to B
    sf::Vector2f n = B->position - A->position;
    
    AABB abox = A->aabb;
    AABB bbox = B->aabb;
    
    // Calculate half extents along x axis for each object
    float a_extent = (abox.max.x - abox.min.x) / 2;
    float b_extent = (bbox.max.x - bbox.min.x) / 2;
    
    // Calculate overlap on x axis
    float x_overlap = a_extent + b_extent - abs(n.x);
    
    // Separating Axis Theorem test on x axis
    if (x_overlap > 0)
    {
        // Calculate half extents along y axis for each object
        float a_extent = (abox.max.y - abox.min.y) / 2;
        float b_extent = (bbox.max.y - bbox.min.y) / 2;
    
        // Calculate overlap on y axis
        float y_overlap = a_extent + b_extent - abs(n.y);
    
        // Separating Axis Theorem test on y axis
        if (y_overlap > 0)
        {
            // Find out which axis is axis of least penetration
            if (x_overlap > y_overlap)
            {
                // Point towards B knowing that n points from A to B
                if (n.x < 0)
                    m->normal = sf::Vector2f(-1, 0);
                else
                    m->normal = sf::Vector2f(1, 0);
                m->penetration = x_overlap;
                return true;
            }
            else
            {
                // Point toward B knowing that n points from A to B
                if (n.y < 0)
                    m->normal = sf::Vector2f(0, -1);
                else
                    m->normal = sf::Vector2f(0, 1);
                m->penetration = y_overlap;
                return true;
            }
        }
    }

    return false;
}


/* Detects an AABB vs Circle collision and generates the manifold if the collision is true */
bool AABBCircleCollision(Manifold *m)
{
    // Setup a couple pointers to each object
    Object *A = m->A;
    Object *B = m->B;
    
    // Vector from A to B
    sf::Vector2f n = B->position - A->position;
    
    // Closest point on A to center of B
    sf::Vector2f closest = n;
    
    // Calculate half extents along each axis of the AABB
    float x_extent = (A->aabb.max.x - A->aabb.min.x) / 2;
    float y_extent = (A->aabb.max.y - A->aabb.min.y) / 2;
    
    // Bound closest point to edges of the AABB
    closest.x = bound(-x_extent, x_extent, closest.x);
    closest.y = bound(-y_extent, y_extent, closest.y);
    
    bool inside = false;
    
    // Circle is inside the AABB, so we need to clamp the circle's center
    // to the closest edge
    if (n == closest)
    {
        inside = true;
    
        // Find closest axis
        if (abs(n.x) > abs(n.y))
        {
            // Clamp to closest extent
            if (closest.x > 0) {
                closest.x = x_extent;
            }
            else {
                closest.x = -x_extent;                
            }
        }
    
        // y axis is shorter
        else
        {
            // Clamp to closest extent
            if (closest.y > 0){
                closest.y = y_extent;
            }
            else {
                closest.y = -y_extent;
            }
        }
    }
    
    sf::Vector2f normal = n - closest;
    float d = length_squared(normal);
    float r = B->radius;
    
    // Return if the radius is shorter than distance to closest point and
    // Circle is not inside the AABB (not a collision)
    if(d > r * r && !inside) {
        return false;
    }
    
    // Avoided sqrt until we needed
    d = sqrt(d);
    
    // Collision normal needs to be flipped to point outside if circle was
    // inside the AABB
    if (inside)
    {
        m->normal = -n / d;
        m->penetration = r - d;
    }
    else
    {
        m->normal = n / d;
        m->penetration = r - d;
    }
    
    return true;
}




void ResolveCollision(Object A, Object B, Manifold *m) 
{
    // Calculate relative velocity
    sf::Vector2f rv = B.velocity - A.velocity;

    // Find the relative velocity in the normal direction
    float rvNormal = dot_product(rv, m->normal);

    // Ensure that the velocities are separating
    if (rvNormal > 0) {
        return;
    }

    // Calculate restitution
    float e = std::min(A.restitution, B.restitution);

    // Calculate impulse scalar
    float j = (-1 + e) * rvNormal;
    j /= A.inv_mass + B.inv_mass;

    // Apply impulse
    sf::Vector2f impulse = j * m->normal;
    A.velocity -= A.inv_mass * impulse;
    B.velocity += B.inv_mass * impulse;



    // Re-calculate relative velocity after normal impulse, to find friction
    // is applied (impulse from first article, this code comes
    // directly thereafter in the same resolve function)
    sf::Vector2f rv = B.velocity - A.velocity;
    
    // Solve for the tangent vector
    sf::Vector2f tangent = rv - dot_product(rv, m->normal) * m->normal;
    normalize(tangent);
    
    // Solve for magnitude to apply along the friction vector
    float jt = -dot_product(rv, tangent);
    jt /= A.inv_mass + B.inv_mass;

    // Coulomb's Law (Ff <= mu * Fn)
    // Use to approximate mu given friction coefficients of each body
    float mu = sqrt(A.material.static_friction * A.material.static_friction + B.material.static_friction * B.material.static_friction);
    
    // Clamp magnitude of friction and create impulse vector
    sf::Vector2f friction_impulse;

    if (abs(jt) < j * mu)
    {
        friction_impulse = jt * tangent;
    } 
    else
    {
        float mu_kinetic = sqrt(A.material.kinetic_friction * A.material.kinetic_friction + B.material.kinetic_friction * B.material.kinetic_friction);
        friction_impulse = -j * tangent * mu_kinetic;
    }
    
    // Apply
    A.velocity -= A.inv_mass * friction_impulse;
    B.velocity += B.inv_mass * friction_impulse;
}





// Positional Correction to prevent sinking
void PositionalCorrection(Object A, Object B, Manifold *m)
{
    const float percent = 0.2; // usually 20% to 80%
    const float slop = 0.01; // usually 0.01 to 0.1
    sf::Vector2f correction = (std::max(m->penetration - slop, 0.0f) / (A.inv_mass + B.inv_mass)) * percent * m->normal;
    A.position -= A.inv_mass * correction;
    B.position += B.inv_mass * correction;
}

