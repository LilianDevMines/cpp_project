#include "Context.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>


const float g = 9.81;
// ------------------------------------------------

Context::Context(int capacity)
{
  this->m_num_particles = 0;
  this->m_particles = new Particle[capacity];
  this->m_num_plans = 0;
  this->m_plans = new Plan[capacity];
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

void Context::addPlan(Vec2 coord1, Vec2 coord2) {
    //Il faut ici ajouter un élément de la classe Particle au tableau m_particles dont le numéro est m_num_particles
    struct Plan plan;
    plan.coord1 = coord1;
    plan.coord2 = coord2;
    this->m_plans[m_num_plans++] = plan;
}

void Context::updatePhysicalSystem(float dt, int num_constraint_relaxation)
{
  applyExternalForce(dt);
  dampVelocities(); // Frottements
  updateExpectedPosition(dt); // Position attendue en fonction de la vitesse
  addDynamicContactConstraints(); // Itérer sur les différents types de contraintes (chevauchement avec un colider / autre particule)
  addStaticContactConstraints();
 
  for(int k=0; k<num_constraint_relaxation; ++k) {
    projectConstraints();
  }

  updateVelocityAndPosition(dt);
  applyFriction();

  deleteContactConstraints();
}

// ------------------------------------------------

void Context::applyExternalForce(float dt)
{
  float weight =  - g*dt;
  for (int i =0;i <this->m_num_particles; i++){
    this->m_particles[i].velocity = Vec2{this->m_particles[i].velocity.x,this->m_particles[i].velocity.y + weight};
    this->m_particles[i].next_pos = Vec2{this->m_particles[i].position.x, this->m_particles[i].position.y +m_particles[i].velocity.y * dt};
  }
}

void Context::dampVelocities()
{
}

void Context::updateExpectedPosition(float dt)
{
  for (int i =0;i <this->m_num_particles; i++){
    this->m_particles[i].position = this->m_particles[i].next_pos;
  }
}

void Context::addDynamicContactConstraints()
{
}

void Context::addStaticContactConstraints()
{
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
