#pragma once

#include "Vec2.h"
#include "Particle.h"
#include "Plan.h"

// ------------------------------------------------

class Context
{
public:
  Context(int capacity);

  int num_particles() const { return m_num_particles; }
  int num_plans() const { return m_num_plans; }

  void addParticle(Vec2 pos, float radius, float mass, Vec2 velocity, int draw_id);
  void addPlan(Vec2 coord1, Vec2 coord2);

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
  int m_num_plans;
  Particle* m_particles;
  Plan* m_plans;

private :
  Vec2 distancetoPlan(Plan plan);
};

// ------------------------------------------------
