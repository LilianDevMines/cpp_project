#pragma once

#include "Vec2.h"
#include "Plan.h"
#include <stdint.h>

// ------------------------------------------------

typedef struct Water {
  Plan surface = {};   // Surface of the water
  float viscosity = 1;   // viscosity in the water
}Water;

// ------------------------------------------------