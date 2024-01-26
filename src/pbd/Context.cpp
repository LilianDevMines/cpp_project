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
void Context::addParticle(Vec2 pos, float radius, float mass, Vec2 velocity, int draw_id) {
    //Il faut ici ajouter un élément de la classe Particle au tableau m_particles dont le numéro est m_num_particles
    struct Particle particle;
    particle.position = pos;
    particle.radius = radius;
    particle.mass = mass;
    particle.velocity = velocity;
    particle.draw_id = draw_id;
    this->m_particles.push_back(particle);
}

void Context::addPlan(Vec2 coord1, Vec2 coord2) {
    Vec2 directeur = Vec2{(coord2.x - coord1.x),(coord2.y - coord1.y)};
    Vec2 normal = Vec2{(-directeur.y)/(std::sqrt(directeur.x*directeur.x + directeur.y*directeur.y)),
                      (directeur.x)/(std::sqrt(directeur.x*directeur.x + directeur.y*directeur.y))};
    this->m_plans.push_back(Plan{coord1, normal});
}

void Context::addWater(Vec2 coord1, Vec2 coord2) {
    Vec2 directeur = Vec2{(coord2.x - coord1.x),(coord2.y - coord1.y)};
    Vec2 normal = Vec2{(-directeur.y)/(std::sqrt(directeur.x*directeur.x + directeur.y*directeur.y)),
                      (directeur.x)/(std::sqrt(directeur.x*directeur.x + directeur.y*directeur.y))};
    this->m_pounds.push_back(Water{Plan{coord1, normal}});
}

void Context::updatePhysicalSystem(float dt, int num_constraint_relaxation)
{
  applyExternalForce(dt);
  dampVelocities(dt); // Frottements
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
    float gravity = g;
    for (auto& particle : this->m_particles) {
        // Forces
        // Weight
        Vec2 Weight{0, - (particle.mass * gravity)};

        // Forces
        Vec2 Force = Vec2{Weight.x, Weight.y};
        
        for (auto& pound : this->m_pounds) {
            if (particle.next_pos.y - particle.radius < pound.surface.point.y) {
                // Archimede
                float h = pound.surface.point.y - particle.next_pos.y - particle.radius;
                Vec2 Archimede = Vec2{0, (pound.density)*(float{3.14}*(h*h)/3)*(3*h-particle.radius)*gravity};

                // update Force
                Force = Vec2{Force.x + Archimede.x, Force.y + Archimede.y};
          }
        }

        // update velocity
        particle.velocity = Vec2{particle.velocity.x + (dt*Force.x)/(particle.mass),
                                  particle.velocity.y + (dt*Force.y)/(particle.mass)};
    }
}

void Context::dampVelocities(float dt)
{
  Vec2 Frottements;
  for (auto& particle : this->m_particles){
    //Frottements
    for (auto& pound : this->m_pounds){
      if (particle.next_pos.y < pound.surface.point.y){
        float k = pound.viscosity*6*3.14*particle.radius;
        Frottements = Vec2{-k*particle.velocity.x, -k*particle.velocity.y};
        //update velocity
        particle.velocity = Vec2{particle.velocity.x + (dt*Frottements.x)/(particle.mass),
                                  particle.velocity.y + (dt*Frottements.y)/(particle.mass)};
      }
    }
  }
}

void Context::updateExpectedPosition(float dt)
{
    for (auto& particle : this->m_particles) {
        particle.next_pos = Vec2{particle.position.x + dt*particle.velocity.x,
                                 particle.position.y + dt*particle.velocity.y};
    }
}

void Context::addDynamicContactConstraints()
{
    for (auto& particle_i : this->m_particles) {
        for (auto& particle_j : this->m_particles) {
            if (&particle_i != &particle_j) {
                //xij
                Vec2 xij = Vec2{particle_i.next_pos.x - particle_j.next_pos.x,
                                particle_i.next_pos.y - particle_j.next_pos.y};

                //C
                float C = norme(xij) - (particle_i.radius + particle_j.radius);

                if (C < 0) {
                  if (particle_i.radius == particle_j.radius) {
                    mergeParticles(particle_i, particle_j);
                  }
                  else {
                      //sigma
                      float sigmai = (1/particle_i.mass)/((1/particle_i.mass) + (1/particle_j.mass))*C;
                      float sigmaj = (1/particle_j.mass)/((1/particle_i.mass) + (1/particle_j.mass))*C;

                      Vec2 deltaposi = Vec2{-sigmai * (xij.x/norme(xij)), -sigmai * (xij.y/norme(xij))};
                      Vec2 deltaposj = Vec2{sigmaj * (xij.x/norme(xij)), sigmaj * (xij.y/norme(xij))};

                      //update next_pos
                      particle_i.next_pos = Vec2{particle_i.next_pos.x + deltaposi.x, particle_i.next_pos.y + deltaposi.y};
                      particle_j.next_pos = Vec2{particle_j.next_pos.x + deltaposj.x, particle_j.next_pos.y + deltaposj.y};
                  }
                }
            }
        }
    }
}

void Context::addStaticContactConstraints()
{
    for (auto& particle : this->m_particles) {
        for (auto& plan : this->m_plans) {
            Vec2 proj_orig = Vec2{particle.position.x - plan.point.x,
                                  particle.position.y - plan.point.y}; 
            Vec2 projection = Vec2{
                particle.position.x - produit_scalaire(proj_orig, plan.normal) * plan.normal.x,
                particle.position.y - produit_scalaire(proj_orig, plan.normal) * plan.normal.y};  

            if (produit_scalaire(Vec2{particle.next_pos.x - projection.x,
                                      particle.next_pos.y - projection.y}, plan.normal) < particle.radius) {
                Vec2 qc = Vec2{
                    particle.next_pos.x - 
                    (produit_scalaire(Vec2{particle.next_pos.x - projection.x,
                                           particle.next_pos.y - projection.y}, plan.normal) * plan.normal.x),
                    particle.next_pos.y - 
                    (produit_scalaire(Vec2{particle.next_pos.x - projection.x,
                                           particle.next_pos.y - projection.y}, plan.normal) * plan.normal.y)};

                float C = produit_scalaire(Vec2{particle.next_pos.x - qc.x,
                                                particle.next_pos.y - qc.y}, plan.normal) - particle.radius;

                Vec2 deltapos = Vec2{-C * plan.normal.x, -C * plan.normal.y};

                particle.next_pos = Vec2{particle.next_pos.x + deltapos.x, particle.next_pos.y + deltapos.y};
            }
        }
    }
}

void Context::projectConstraints()
{
}

void Context::updateVelocityAndPosition(float dt)
{
    for (auto& particle : this->m_particles) {
        //update velocity
        particle.velocity = Vec2{(particle.next_pos.x - particle.position.x)/dt,
                                  (particle.next_pos.y - particle.position.y)/dt};
        //update position
        particle.position = particle.next_pos;
    }
}                    


void Context::mergeParticles(Particle& particle_i, Particle& particle_j)
{
    // Merge particles
    particle_j.mass += particle_i.mass;
    particle_j.radius = particle_i.radius + particle_j.radius;
    particle_j.velocity = Vec2{(particle_i.velocity.x * particle_i.mass + particle_j.velocity.x * particle_j.mass) / (particle_i.mass + particle_j.mass),
                                (particle_i.velocity.y * particle_i.mass + particle_j.velocity.y * particle_j.mass) / (particle_i.mass + particle_j.mass)};

    // Remove the merged particle from the list
    // We need to find the particle in the list by iterating and using references
    for (auto it = this->m_particles.begin(); it != this->m_particles.end(); ++it) {
        if (&(*it) == &particle_i) {
            // Remove the merged particle from the list

            this->m_dead_particles.push_back(it->draw_id);
            this->m_particles.erase(it);
            break;
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
//Tools for calculus