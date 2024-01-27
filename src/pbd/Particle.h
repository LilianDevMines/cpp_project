#pragma once

#include "Vec2.h"
#include <stdint.h>

// ------------------------------------------------

struct Particle
{
  Vec2 position; // current position of the particle
  Vec2 next_pos; // only used during update loop
  Vec2 velocity; // current velocity of the particle
  float mass;
  float radius; // radius of the particle
  int draw_id;  // id used to identify drawing element associated to the sphere
  bool merged;
};

// ------------------------------------------------
