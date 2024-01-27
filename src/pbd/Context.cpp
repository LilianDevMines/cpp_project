#include "Context.h"

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <list>

const float g = 9.81;
// ------------------------------------------------

Context::Context()
{
}

// ------------------------------------------------
void Context::addParticle(Vec2 pos, float radius, float mass, Vec2 velocity, int draw_id)
{
  // Il faut ici ajouter un élément de la classe Particle au tableau m_particles dont le numéro est m_num_particles
  struct Particle particle;
  particle.position = pos;
  particle.radius = radius;
  particle.mass = mass;
  particle.velocity = velocity;
  particle.draw_id = draw_id;
  particle.merged = false;
  this->m_particles.push_back(particle);
}

void Context::addPlan(Vec2 coord1, Vec2 coord2, int draw_id)
{
  Vec2 directeur;
  directeur.x = coord2.x - coord1.x;
  directeur.y = coord2.y - coord1.y;
  Vec2 normal;
  normal.x = (-directeur.y) / (std::sqrt(directeur.x * directeur.x + directeur.y * directeur.y));
  normal.y = (directeur.x) / (std::sqrt(directeur.x * directeur.x + directeur.y * directeur.y));
  Plan new_plan;
  new_plan.normal = normal;
  new_plan.point = coord1;
  new_plan.draw_id = draw_id;
  this->m_plans.push_back(new_plan);
}

void Context::addWater(Vec2 coord1, Vec2 coord2)
{
  Vec2 directeur;
  directeur.x = coord2.x - coord1.x;
  directeur.y = coord2.y - coord1.y;
  Vec2 normal;
  normal.x = (-directeur.y) / (std::sqrt(directeur.x * directeur.x + directeur.y * directeur.y));
  normal.y = (directeur.x) / (std::sqrt(directeur.x * directeur.x + directeur.y * directeur.y));
  Plan new_plan;
  new_plan.normal = normal;
  new_plan.point = coord1;
  Water new_water;
  new_water.surface = new_plan;
  this->m_pounds.push_back(new_water);
}

void Context::updatePhysicalSystem(float dt, int num_constraint_relaxation)
{
  applyExternalForce(dt);
  dampVelocities(dt);         // Frottements
  updateExpectedPosition(dt); // Position attendue en fonction de la vitesse
  mergeParticles();           // Merged particule if necessary after having applied forces
  addDynamicContactConstraints();
  addStaticContactConstraints(); // Itérer sur les différents types de contraintes (chevauchement avec un colider / autre particule)

  for (int k = 0; k < num_constraint_relaxation; ++k)
  {
    projectConstraints();
  }

  updateVelocityAndPosition(dt);
  applyFriction();

  deleteContactConstraints();
}

// ------------------------------------------------

void Context::applyExternalForce(float dt)
{
  for (std::list<Particle>::iterator it = this->m_particles.begin(); it != this->m_particles.end(); ++it)
  {
    Particle &particle = *it;
    // Forces
    // Weight
    Vec2 Weight;
    Weight.x = 0;
    Weight.y = -(particle.mass * g);

    // Forces
    Vec2 Force;
    Force.x = Weight.x;
    Force.y = Weight.y;

    for (std::list<Water>::iterator it_pound = this->m_pounds.begin(); it_pound != this->m_pounds.end(); ++it_pound)
    {
      Water &pound = *it_pound;
      if (particle.next_pos.y - particle.radius < pound.surface.point.y)
      {
        // Archimede
        float h = pound.surface.point.y - particle.next_pos.y - particle.radius;
        Vec2 Archimede;
        Archimede.x = 0;
        Archimede.y = (pound.density) * (3.14 * (h * h) / 3) * (3 * h - particle.radius) * g;

        // update Force
        Force.x += Archimede.x;
        Force.y += Archimede.y;
      }
    }

    // update velocity
    Vec2 new_velocity;
    new_velocity.x = particle.velocity.x + (dt * Force.x) / (particle.mass);
    new_velocity.y = particle.velocity.y + (dt * Force.y) / (particle.mass);
    particle.velocity = new_velocity;
  }
}

void Context::dampVelocities(float dt)
{

  for (std::list<Particle>::iterator it = this->m_particles.begin(); it != this->m_particles.end(); ++it)
  {
    Particle &particle = *it;
    for (std::list<Water>::iterator it_pound = this->m_pounds.begin(); it_pound != this->m_pounds.end(); ++it_pound)
    {
      Water &pound = *it_pound;
      if (particle.next_pos.y < pound.surface.point.y)
      {
        float k = pound.viscosity * 6 * 3.14 * particle.radius;

        // Damp definition
        Vec2 Frottements;
        Frottements.x = -k * particle.velocity.x;
        Frottements.y = -k * particle.velocity.y;

        // update velocity
        particle.velocity.x = particle.velocity.x + (dt * Frottements.x) / (particle.mass);
        particle.velocity.y = particle.velocity.y + (dt * Frottements.y) / (particle.mass);
      }
    }
  }
}

void Context::updateExpectedPosition(float dt)
{
  for (std::list<Particle>::iterator it = this->m_particles.begin(); it != this->m_particles.end(); ++it)
  {
    Particle &particle = *it;
    particle.next_pos.x = particle.position.x + dt * particle.velocity.x;
    particle.next_pos.y = particle.position.y + dt * particle.velocity.y;
  }
}

void Context::addDynamicContactConstraints()
{
  for (std::list<Particle>::iterator it_i = this->m_particles.begin(); it_i != this->m_particles.end(); ++it_i)
  {
    Particle &particle_i = *it_i;
    for (std::list<Particle>::iterator it_j = this->m_particles.begin(); it_j != this->m_particles.end(); ++it_j)
    {
      Particle &particle_j = *it_j;
      if (&particle_i != &particle_j)
      {
        // xij
        Vec2 xij;
        xij.x = particle_i.next_pos.x - particle_j.next_pos.x;
        xij.y = particle_i.next_pos.y - particle_j.next_pos.y;
        float norm_xij = norme(xij);

        // C
        float C = norm_xij - (particle_i.radius + particle_j.radius);
        if (C <= 0)
        {
          // Check if the particule is in contact with a merged particule
          if (particle_i.merged || particle_j.merged)
          {
            particle_i.merged = true;
            particle_j.merged = true;
          }

          // sigma
          float sigmai = (1 / particle_i.mass) / ((1 / particle_i.mass) + (1 / particle_j.mass)) * C;
          float sigmaj = (1 / particle_j.mass) / ((1 / particle_i.mass) + (1 / particle_j.mass)) * C;

          Vec2 deltaposi;
          deltaposi.x = -sigmai * (xij.x / norm_xij);
          deltaposi.y = -sigmai * (xij.y / norm_xij);
          Vec2 deltaposj;
          deltaposj.x = sigmaj * (xij.x / norm_xij);
          deltaposj.y = sigmaj * (xij.y / norm_xij);

          // update next_pos
          particle_i.next_pos.x += deltaposi.x;
          particle_i.next_pos.y += deltaposi.y;

          particle_j.next_pos.x += deltaposj.x;
          particle_j.next_pos.y += deltaposj.y;
        }
      }
    }
  }
}

void Context::addStaticContactConstraints()
{
  for (std::list<Particle>::iterator it_particle = this->m_particles.begin(); it_particle != this->m_particles.end(); ++it_particle)
  {
    Particle &particle = *it_particle;
    for (std::list<Plan>::iterator it_plan = this->m_plans.begin(); it_plan != this->m_plans.end(); ++it_plan)
    {
      Plan &plan = *it_plan;
      Vec2 proj_orig;
      proj_orig.x = particle.position.x - plan.point.x;
      proj_orig.y = particle.position.y - plan.point.y;
      // Calcule P - Q où P à projeter et Q dans le plan
      Vec2 projection;
      projection.x = particle.position.x - produit_scalaire(proj_orig, plan.normal) * plan.normal.x;
      projection.y = particle.position.y - produit_scalaire(proj_orig, plan.normal) * plan.normal.y;
      // Calcule la projection de P sur le plan
      Vec2 vector;
      vector.x = particle.next_pos.x - projection.x;
      vector.y = particle.next_pos.y - projection.y;

      if (produit_scalaire(vector, plan.normal) < particle.radius)
      {
        Vec2 qc;
        qc.x = particle.next_pos.x - (produit_scalaire(vector, plan.normal) * plan.normal.x);
        qc.y = particle.next_pos.y - (produit_scalaire(vector, plan.normal) * plan.normal.y);

        Vec2 vector2;
        vector2.x = particle.next_pos.x - qc.x;
        vector2.y = particle.next_pos.y - qc.y;
        float C = produit_scalaire(vector2, plan.normal) - particle.radius;

        Vec2 deltapos;
        deltapos.x = -C * plan.normal.x;
        deltapos.y = -C * plan.normal.y;
        // Update next position
        particle.next_pos.x += deltapos.x;
        particle.next_pos.y += deltapos.y;
      }
    }
  }
}

void Context::projectConstraints()
{
}

void Context::updateVelocityAndPosition(float dt)
{
  for (std::list<Particle>::iterator it = this->m_particles.begin(); it != this->m_particles.end(); ++it)
  {
    Particle &particle = *it;

    // update velocity
    if (particle.merged)
    {
      particle.velocity.x = 0;
      particle.velocity.y = 0;
      std::cout << particle.draw_id << std::endl;
      particle.merged = false;
    }
    else
    {
      particle.velocity.x = (particle.next_pos.x - particle.position.x) / dt;
      particle.velocity.y = (particle.next_pos.y - particle.position.y) / dt;
    }

    // update position
    particle.position = particle.next_pos;
  }
}

void Context::mergeParticles()
{
  for (std::list<Particle>::iterator it_i = this->m_particles.begin(); it_i != this->m_particles.end(); ++it_i)
  {
    Particle &particle_i = *it_i;
    for (std::list<Particle>::iterator it_j = this->m_particles.begin(); it_j != this->m_particles.end(); ++it_j)
    {
      Particle &particle_j = *it_j;
      if (&particle_i != &particle_j)
      {
        // xij
        Vec2 xij;
        xij.x = particle_i.next_pos.x - particle_j.next_pos.x;
        xij.y = particle_i.next_pos.y - particle_j.next_pos.y;

        // C
        float C = norme(xij) - (particle_i.radius + particle_j.radius);
        if (C <= 0)
        {
          if (particle_i.radius == particle_j.radius)
          {
            // Merge particles
            particle_i.mass = particle_i.mass + particle_j.mass;
            particle_i.radius = particle_i.radius + particle_j.radius;
            particle_i.velocity.x = (particle_i.velocity.x * particle_i.mass + particle_j.velocity.x * particle_j.mass) / (particle_i.mass + particle_j.mass);
            particle_i.velocity.y = (particle_i.velocity.y * particle_i.mass + particle_j.velocity.y * particle_j.mass) / (particle_i.mass + particle_j.mass);
            particle_i.position.x = (particle_i.position.x + particle_j.position.x) / 2;
            particle_i.position.y = (particle_i.position.y + particle_j.position.y) / 2;
            // Notify that the paticle has been merged
            particle_i.merged = true;
            // Remove the particle unused
            it_j = this->m_particles.erase(it_j);
          }
        }
      }
    }
  }
}

void Context::applyFriction()
{
}

void Context::deleteContactConstraints()
{
}

// ------------------------------------------------
// Tools for calculus