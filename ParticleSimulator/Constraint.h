#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <glm/glm.hpp>
#include "Particle.h"
#include <vector>

//spring constraints
class SpringConstraint {
public:
	Particle* u;
	Particle* v;
	double k;
	double rest_length;

	SpringConstraint(Particle* u, Particle* v, double k, double rest_length);
};

void applySpringForce(SpringConstraint* constraint);

#endif //CONSTRAINT_H
