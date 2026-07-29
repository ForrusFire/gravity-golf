#ifndef SCENE_HPP
#define SCENE_HPP


#include "PhysicsEngine/object.hpp"


class Scene
{
public:
  Scene( Vec2 gravity, real dt );
  ~Scene( );
 
  void SetGravity( Vec2 gravity )
  void SetDT( real dt )
 
  Body *CreateBody( ShapeInterface *shape, BodyDef def )
 
  // Inserts a body into the scene and initializes the body (computes mass).
  //void InsertBody( Body *body )
 
  // Deletes a body from the scene
  void RemoveBody( Body *body )
 
  // Updates the scene with a single timestep
  void Step( void )
 
  float GetDT( void )
  LinkedList *GetBodyList( void )
  Vec2 GetGravity( void )
  void QueryAABB( CallBackQuery cb, const AABB& aabb )
  void QueryPoint( CallBackQuery cb, const Point2& point )
 
private:
  float dt     // Timestep in seconds
  float inv_dt // Inverse timestep in sceonds
  LinkedList body_list
  uint32 body_count
  Vec2 gravity
  bool debug_draw
  BroadPhase broadphase
};


#endif