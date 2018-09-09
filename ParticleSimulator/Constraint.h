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
	double rest_length_orig;
	double (*restLengthModifier) (int);

	SpringConstraint(Particle* u, Particle* v, double k, double rest_length);
	static SpringConstraint * SpringConstraintWithModifier(Particle* u, Particle* v, double k, double rest_length, double(*restLengthModifier) (int));
};

void updateRestLength(SpringConstraint* constraint, double time);

void applySpringForce(SpringConstraint* constraint);

class HardConstraint{
public:
	Particle *u;
	Particle* v;
	double rest_length;

	HardConstraint(Particle *u, Particle *v, double rest_length);
};

void applyHardConstraint(HardConstraint* constraint);

namespace Constraint {
	double sinConstraint(int);
}

#endif //CONSTRAINT_H
