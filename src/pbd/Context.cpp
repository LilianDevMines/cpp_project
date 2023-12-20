#include "Context.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>

// ------------------------------------------------

Context::Context(int capacity)
{
  this->m_num_particles = 0;
  this->m_particles = new Particle[capacity];
}

// ------------------------------------------------
void Context::addParticle(Vec2 pos, float radius, float mass, Vec2 velocity, int draw_id) {
    //Il faut ici ajouter un élément de la classe Particle au tableau m_particles dont le numéro est m_num_particles
    struct Particle particle;
    particle.position = pos;
    particle.radius = radius;
    particle.mass = mass;
    particle.velocity = velocity;
    particle.draw_id = draw_id;
    this->m_particles[m_num_particles++] = particle;
}
// ------------------------------------------------

void Context::updatePhysicalSystem(float dt, int num_constraint_relaxation)
{
#define FIXED_VELOCITY
#ifdef FIXED_VELOCITY
  // TODO For testing purposes, add an update of position based on particle velocity
#else
  applyExternalForce(dt);
  dampVelocities();
  updateExpectedPosition(dt);
  addDynamicContactConstraints();
  addStaticContactConstraints();
 
  for(int k=0; k<num_constraint_relaxation; ++k) {
    projectConstraints();
  }

  updateVelocityAndPosition(dt);
  applyFriction();

  deleteContactConstraints();
#endif
}

// ------------------------------------------------

void Context::applyExternalForce(float dt)
{
<<<<<<< Updated upstream
=======
  float weight =  - g*dt;
  for (int i =0;i <this->m_num_particles; i++){
    this->m_particles[i].velocity = Vec2{this->m_particles[i].velocity.x,this->m_particles[i].velocity.y + weight};
    this->m_particles[i].next_pos = Vec2{this->m_particles[i].position.x, this->m_particles[i].position.y +m_particles[i].velocity.y * dt};
  }
  /*for (int i =0;i <this->m_num_particles; i++){
    for (int p =0;p <this->m_num_plans; p++){
      if (this->m_particles[i].next_pos.y < this->m_plans[p].coord2.y){
        this->m_particles[i].next_pos.y = this->m_plans[p].coord2.y + this->m_particles[i].radius;
      }
    }
  }*/
>>>>>>> Stashed changes
}

void Context::dampVelocities()
{
}

void Context::updateExpectedPosition(float dt)
{
<<<<<<< Updated upstream
=======
  for (int i =0;i <this->m_num_particles; i++){
    for (int p =0;p <this->m_num_plans; p++){
      if (this->m_particles[i].next_pos.y < this->m_plans[p].coord2.y){
        this->m_particles[i].next_pos.y = this->m_plans[p].coord2.y + this->m_particles[i].radius;
      }
      this->m_particles[i].position = this->m_particles[i].next_pos;
    }
  }
>>>>>>> Stashed changes
}

void Context::addDynamicContactConstraints()
{
}

void Context::addStaticContactConstraints(){
  /*for (int i =0;i <this->m_num_particles; i++){
    for (int p =0;p <this->m_num_plans; p++){
      if (this->m_particles[i].next_pos.y < this->m_plans[p].coord2.y){
        this->m_particles[i].next_pos.y = this->m_plans[p].coord2.y + this->m_particles[i].radius;
      }
    }
  }*/
}

void Context::projectConstraints()
{
}

void Context::updateVelocityAndPosition(float dt)
{
}

void Context::applyFriction()
{
}

void Context::deleteContactConstraints()
{
}

// ------------------------------------------------
