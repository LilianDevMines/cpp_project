#pragma once

#include "Vec2.h"
#include "Plan.h"
#include <stdint.h>

// ------------------------------------------------

typedef struct Water
{
  Plan surface;    // Surface of the water
  float viscosity; // viscosity in the water
  float density;   // density of the water
} Water;

// ------------------------------------------------