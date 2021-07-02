#include "physics.hpp"


/* The main loop (with clock) will eventually be in the main.cpp file. 

This file (physics.cpp) will contain
 separate high-level functions for performing a physics engine step, and rendering the engine to the screen


scene.cpp will feed to physics.cpp and perform all of the broad phase collision collection as well as the sympletic euler, adding and removing bodies
collision.cpp generates manifold information (normal vec and penetration) between colliding objects and performs narrow phase collision detection
manifold.cpp contains contact information. detected collisions and their respective manifolds will be resolved through impulses in this file

body.cpp contains the actual bodies that can possibly collide and their data
shape.cpp will contain the various shapes and feeds to body.cpp

math.cpp contains math, simple import
*/
void MainPhysicsLoop(void)
{
    const float fps = 100;
    const float dt = 1 / fps;
    float accumulator = 0;
    
    // In units seconds
    float frameStart = GetCurrentTime();
    
    // main loop
    while (true)
    {
        const float currentTime = GetCurrentTime();
        
        // Store the time elapsed since the last frame began
        accumulator += currentTime - frameStart();
        
        // Record the starting of this frame
        frameStart = currentTime;
        
        // Avoid spiral of death and clamp dt, thus clamping
        // how many times the UpdatePhysics can be called in
        // a single game loop.
        if (accumulator > 0.2f) {
            accumulator = 0.2f;
        }
        
        while(accumulator > dt)
        {
            UpdatePhysics(dt);
            accumulator -= dt;
        }

        const float alpha = accumulator / dt;
        
        RenderGame(alpha);
    }
}

 
void RenderGame(float alpha)
{
    for shape in game do
    {
        // calculate an linearly interpolated transform for rendering
        Transform i = shape.previous * alpha + shape.current * (1.0f - alpha);
        shape.previous = shape.current;
        shape.Render(i);
    }

}
