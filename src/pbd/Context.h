#pragma once

#include "Vec2.h"
#include "Particle.h"

// ------------------------------------------------

class Context
{
public:
  Context(int capacity);

  int num_particles() const { return m_num_particles; }

  void addParticle(Vec2 pos, float radius, float mass, Vec2 velocity, int draw_id);

  const Particle& particle(int id) const { return m_particles[id]; }

  void updatePhysicalSystem(float dt, int num_constraint_relaxation);
  
private:
  // Methods below are called by updatePhysicalSystem
  void applyExternalForce(float dt);
  void dampVelocities();
  void updateExpectedPosition(float dt);
  void addDynamicContactConstraints();
  void addStaticContactConstraints();
  void projectConstraints();
  void updateVelocityAndPosition(float dt);
  void applyFriction();
  void deleteContactConstraints();

private:
  int m_num_particles;
  Particle* m_particles;
};

// ------------------------------------------------
