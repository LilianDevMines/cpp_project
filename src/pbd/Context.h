#pragma once

#include "pbd/Vec2.h"
#include "pbd/Particle.h"

// ------------------------------------------------

class Context
{
public:
  Context(int capacity);

  int num_particles() const { return m_num_particles; }

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
