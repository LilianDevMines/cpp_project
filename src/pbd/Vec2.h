#pragma once

#include <cmath>
// ------------------------------------------------

struct Vec2
{
  float x;
  float y;
};

// ------------------------------------------------
// tools for calculus

float produit_scalaire(Vec2 a, Vec2 b);

float norme(Vec2 a);