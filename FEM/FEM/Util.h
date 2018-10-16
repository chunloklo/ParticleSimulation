#pragma once
#include <glm/glm.hpp>
#include <vector>

# define M_PI           (3.14159265358979323846)  /* pi */

void drawCircle(double x, double y, double z, float radius);

void drawSphere(double x, double y, double z, float radius);

//void drawParticles(std::vector<Particle*>& particles);

glm::dvec3 elementwiseMax(glm::dvec3 x, glm::dvec3 y);

glm::dvec3 elementwiseMin(glm::dvec3 u, glm::dvec3 v);

void drawCylinderEndpoints(glm::dvec3 u, glm::dvec3 v, float r);

void drawLineEndpoints(glm::dvec3 u, glm::dvec3 v);

//void drawSpringConstraints(std::vector<SpringConstraint*> &springConstraints);
//void drawHardConstraints(std::vector<HardConstraint*> &hardConstraints);

