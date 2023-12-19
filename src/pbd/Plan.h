#pragma once

#include "Vec2.h"
#include <stdint.h>

// ------------------------------------------------

struct Plan {
  Vec2 coord1 = {};   // Point 1 of the line
  Vec2 coord2 = {};   // Point 2 of the line
  //int draw_id   = 0;    // id used to identify drawing element associated to the sphere
};

// ------------------------------------------------
