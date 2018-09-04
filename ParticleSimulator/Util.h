#ifndef UTIL_H
#define UTIL_H

#include <glm/glm.hpp>
#include "Particle.h"
#include <vector>

void drawCircle(double x, double y, double z, float radius);

void drawSphere(double x, double y, double z, float radius);

void drawParticles(std::vector<Particle*>& particles);

glm::dvec3 elementwiseMax(glm::dvec3 x, glm::dvec3 y);

glm::dvec3 elementwiseMin(glm::dvec3 u, glm::dvec3 v);

#endif //UTIL_H
