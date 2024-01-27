#pragma once

#include "Vec2.h"
#include <stdint.h>

// ------------------------------------------------

typedef struct Plan
{
  Vec2 point;  // Point 1 of the line
  Vec2 normal; // Normal of the line
  int draw_id; // id used to identify drawing element associated to the sphere
} Plan;

// ------------------------------------------------
