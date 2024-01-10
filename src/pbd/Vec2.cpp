#include "pbd/Vec2.h"

// ------------------------------------------------

// ------------------------------------------------
//tools for calculus

float produit_scalaire(Vec2 a, Vec2 b){
  return a.x*b.x + a.y*b.y;
}

float norme(Vec2 a){
  return std::sqrt(produit_scalaire(a,a));
}