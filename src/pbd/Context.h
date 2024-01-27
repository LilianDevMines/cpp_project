#pragma once

#include "Vec2.h"
#include "Particle.h"
#include "Plan.h"
#include "Water.h"
#include <stdint.h>
#include <list>
#include <stdexcept>
#include <set>

// ------------------------------------------------

class Context
{
public:
  Context();

  int num_particles() const { return m_particles.size(); }
  int num_plans() const { return m_plans.size(); }

  void addParticle(Vec2 pos, float radius, float mass, Vec2 velocity, int draw_id);
  void addPlan(Vec2 coord1, Vec2 coord2, int draw_id);
  void addWater(Vec2 coord1, Vec2 coord2);

  const Particle &particle(int id) const
  {
    int i = 0;
    for (std::list<Particle>::const_iterator it = this->m_particles.begin(); it != this->m_particles.end(); ++it)
    {
      if (i == id)
      {
        return *it;
      }
      i++;
    }
    throw std::invalid_argument("Il n'y a pas de particule avec cet id");
  }

  void updatePhysicalSystem(float dt, int num_constraint_relaxation);

private:
  // Methods below are called by updatePhysicalSystem
  void applyExternalForce(float dt);
  void dampVelocities(float dt);
  void updateExpectedPosition(float dt);
  void addDynamicContactConstraints();
  void addStaticContactConstraints();
  void projectConstraints();
  void updateVelocityAndPosition(float dt);
  void mergeParticles();
  void applyFriction();
  void deleteContactConstraints();
  // Data
  std::list<Particle> m_particles; // List of particles
  std::list<Plan> m_plans;         // List of plans
  std::list<Water> m_pounds;       // List of water pounds
};

// ------------------------------------------------
