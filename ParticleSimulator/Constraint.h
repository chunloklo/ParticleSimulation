#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <glm/glm.hpp>
#include "Particle.h"
#include <vector>

namespace Constraint {
	class Constraint {
	public:
		virtual float eval(float time) = 0;
	};

	class SinConstraint : public Constraint {
	public:
		float amplitude;
		float frequency;
		float phase_offset;
		float base_length;

		SinConstraint(float amplitude, float frequency, float phase_offset, float base_length);

		float eval(float time);

	};
}


//spring constraints
class SpringConstraint {
public:
	Particle* u;
	Particle* v;
	double k;
	double rest_length;
	Constraint::Constraint* constraint;

	SpringConstraint(Particle* u, Particle* v, double k, double rest_length);
	static SpringConstraint * SpringConstraintWithModifier(Particle* u, Particle* v, double k, double rest_length, Constraint::Constraint* constraint);
};

void updateRestLength(SpringConstraint* constraint, float time);

void applySpringForce(SpringConstraint* constraint);

class HardConstraint{
public:
	Particle *u;
	Particle* v;
	double rest_length;
	Constraint::Constraint* constraint;

	HardConstraint(Particle *u, Particle *v, double rest_length, Constraint::Constraint* constraint);
};

void applyHardConstraint(HardConstraint* constraint);

void updateHardConstraints(HardConstraint* constraint, float time);


#endif //CONSTRAINT_H
