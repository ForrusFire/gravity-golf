#include "scene.hpp"


// Sympletic (Semi-Implicit) Euler
void SympleticEuler(Object *A, float dt)
{
    // v += (1/m * F) * dt
    A->velocity += (A->mass_data.inv_mass * A->force) * dt;
    A->angular_velocity += (A->mass_data.inv_inertia * A->torque) * dt;

    // x += v * dt
    A->position += A->velocity * dt;
    A->theta += A->angular_velocity * dt;
}


// Generates the broad phase collisions list.
// All previous collisions are cleared when this function is called.
void GenerateBroadCollisions(void)
{
    // Generate new collision info
    collisions.clear();

    for(int i = 0; i < objects.size(); i++)
    {
        Object *A = objects[i];

        for(int j = i + 1; j < objects.size(); j++)
        {
            Object *B = objects[j];
            if (A->mass_data.inv_mass == 0 && B->mass_data.inv_mass == 0) {
                continue;
            }

            Manifold m(A, B);
            m.Solve();

            if (m.contact_count) {
                collisions.emplace_back(m);
            }
        }
    }
}