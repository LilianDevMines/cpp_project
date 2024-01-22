#include "Context.h"

#include <stdio.h>
#include <iostream>
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
  this->m_num_pounds = 0;
  this->m_pounds = new Water[capacity];
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
    Vec2 directeur = Vec2{(coord2.x - coord1.x),(coord2.y - coord1.y)};
    Vec2 normal = Vec2{(-directeur.y)/(std::sqrt(directeur.x*directeur.x + directeur.y*directeur.y)),
                      (directeur.x)/(std::sqrt(directeur.x*directeur.x + directeur.y*directeur.y))};
    this->m_plans[m_num_plans++] = Plan{coord1, normal};
}

void Context::addWater(Vec2 coord1, Vec2 coord2) {
    //Il faut ici ajouter un élément de la classe Particle au tableau m_particles dont le numéro est m_num_particles
    Vec2 directeur = Vec2{(coord2.x - coord1.x),(coord2.y - coord1.y)};
    Vec2 normal = Vec2{(-directeur.y)/(std::sqrt(directeur.x*directeur.x + directeur.y*directeur.y)),
                      (directeur.x)/(std::sqrt(directeur.x*directeur.x + directeur.y*directeur.y))};
    this->m_pounds[m_num_pounds++] = Water{Plan{coord1, normal}};
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
  for (int i =0;i <this->m_num_particles; i++){
    //Forces
    //Weight
    Vec2 Weight{0, - (this->m_particles[i].mass *g)};

    //Forces
    Vec2 Force = Vec2{Weight.x, Weight.y};
    
    for (int w = 0;w<this->m_num_pounds;w++){
      if (this->m_particles[i].next_pos.y - this->m_particles[i].radius < this->m_pounds[w].surface.point.y){
        //Archimede
        float h = this->m_pounds[w].surface.point.y - this->m_particles[i].next_pos.y - this->m_particles[i].radius;
        Vec2 Archimede = Vec2{0, (this->m_pounds[w].density)*(3.14*(h*h)/3)*(3*h-this->m_particles[i].radius) *g};

        //update Force
        Force = Vec2{Force.x + Archimede.x, Force.y + Archimede.y};
      }
    }

    //update velocity
    this->m_particles[i].velocity = Vec2{this->m_particles[i].velocity.x + (dt*Force.x)/(this->m_particles[i].mass),
                                        this->m_particles[i].velocity.y + (dt*Force.y)/(this->m_particles[i].mass)};
  }
}

void Context::dampVelocities(float dt)
{
  Vec2 Frottements;
  for (int i =0;i <this->m_num_particles; i++){
    //Frottements
    for (int w = 0;w<this->m_num_pounds;w++){
      if (this->m_particles[i].next_pos.y < this->m_pounds[w].surface.point.y){
        float k = m_pounds[w].viscosity*6*3.14*this->m_particles[i].radius;
        Frottements = Vec2{-k*this->m_particles[i].velocity.x, -k*this->m_particles[i].velocity.y};
        //update velocity
        this->m_particles[i].velocity = Vec2{this->m_particles[i].velocity.x + (dt*Frottements.x)/(this->m_particles[i].mass),
                                        this->m_particles[i].velocity.y + (dt*Frottements.y)/(this->m_particles[i].mass)};
      }
    }
  }
}

void Context::updateExpectedPosition(float dt)
{
  for (int i =0;i <this->m_num_particles; i++){
    this->m_particles[i].next_pos = Vec2{this->m_particles[i].position.x + dt*this->m_particles[i].velocity.x,
                                        this->m_particles[i].position.y + dt*this->m_particles[i].velocity.y};
  }
}

void Context::addDynamicContactConstraints()
{ 
  for (int i =0;i <this->m_num_particles; i++){
    for (int j =0;j <this->m_num_particles; j++){
      if (i!=j){
        //xij
        Vec2 xij = Vec2{this->m_particles[i].next_pos.x - this->m_particles[j].next_pos.x,
                        this->m_particles[i].next_pos.y - this->m_particles[j].next_pos.y};

        //C
        float C = norme(xij) - (this->m_particles[i].radius + this->m_particles[j].radius);

        if (C < 0){

        //sigma
        float sigmai = (1/this->m_particles[i].mass)/((1/this->m_particles[i].mass) + (1/this->m_particles[j].mass))*C;
        float sigmaj = (1/this->m_particles[j].mass)/((1/this->m_particles[i].mass) + (1/this->m_particles[j].mass))*C;

        Vec2 deltaposi = Vec2{-sigmai * (xij.x/norme(xij)), -sigmai * (xij.y/norme(xij))};
        Vec2 deltaposj = Vec2{sigmaj * (xij.x/norme(xij)), sigmaj * (xij.y/norme(xij))};

          //update next_pos
          this->m_particles[i].next_pos = Vec2{this->m_particles[i].next_pos.x + deltaposi.x, this->m_particles[i].next_pos.y + deltaposi.y};
          this->m_particles[j].next_pos = Vec2{this->m_particles[j].next_pos.x + deltaposj.x, this->m_particles[j].next_pos.y + deltaposj.y};
        }
      }
    }
  }
}

void Context::addStaticContactConstraints(){
  for (int i =0;i <this->m_num_particles; i++){
    for (int p =0;p <this->m_num_plans; p++){
      Vec2 proj_orig = Vec2{this->m_particles[i].position.x - this->m_plans[p].point.x,
                       this->m_particles[i].position.y - this->m_plans[p].point.y}; //Calcule P - Q où P à projeter et Q dans le plan
      Vec2 projection = Vec2{
        this->m_particles[i].position.x - produit_scalaire(proj_orig, this->m_plans[p].normal) * this->m_plans[p].normal.x,
        this->m_particles[i].position.y - produit_scalaire(proj_orig, this->m_plans[p].normal) * this->m_plans[p].normal.y};  
        //Calcule la projection de P sur le plan
      if (produit_scalaire(Vec2{this->m_particles[i].next_pos.x - projection.x,
                               this->m_particles[i].next_pos.y - projection.y},this->m_plans[p].normal) < this->m_particles[i].radius){
        Vec2 qc = Vec2{
          this->m_particles[i].next_pos.x - 
        (produit_scalaire(Vec2{this->m_particles[i].next_pos.x - projection.x,
                                this->m_particles[i].next_pos.y - projection.y},this->m_plans[p].normal)* this->m_plans[p].normal.x),
        this->m_particles[i].next_pos.y - 
        (produit_scalaire(Vec2{this->m_particles[i].next_pos.x - projection.x,
                                this->m_particles[i].next_pos.y - projection.y},this->m_plans[p].normal)* this->m_plans[p].normal.y)};

        float C = produit_scalaire(Vec2{this->m_particles[i].next_pos.x - qc.x,
                                this->m_particles[i].next_pos.y - qc.y},this->m_plans[p].normal) - this->m_particles[i].radius;

        Vec2 deltapos = Vec2{-C * this->m_plans[p].normal.x, -C * this->m_plans[p].normal.y};

        this->m_particles[i].next_pos = Vec2{this->m_particles[i].next_pos.x + deltapos.x, this->m_particles[i].next_pos.y + deltapos.y};
        }
    }
  }
}

void Context::projectConstraints()
{
}

void Context::updateVelocityAndPosition(float dt)
{
  for (int i =0;i <this->m_num_particles; i++){
      //update velocity
      this->m_particles[i].velocity = Vec2{(this->m_particles[i].next_pos.x - this->m_particles[i].position.x)/dt,
                                            (this->m_particles[i].next_pos.y - this->m_particles[i].position.y)/dt};
      //update position
      this->m_particles[i].position = this->m_particles[i].next_pos;
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