#include "Constraint.h"
#include <glm/glm.hpp>

//either- between particles, hard environmental constraints, or
# define M_PI           (3.14159265358979323846)  /* pi */

SpringConstraint::SpringConstraint(Particle * u, Particle * v, double k, double rest_length)
	: u(u), v(v), k(k), rest_length(rest_length), constraint(NULL) {};

SpringConstraint* SpringConstraint::SpringConstraintWithModifier(Particle* u, Particle* v, double k, double rest_length, Constraint::Constraint* constraint) {
	SpringConstraint* springConstraint = new SpringConstraint(u, v, k, rest_length);
	springConstraint->constraint = constraint;
	return springConstraint;
}

void applySpringForce(SpringConstraint* constraint) {
	glm::dvec3 dv = constraint->u->pos - constraint->v->pos;
	double drest_length = glm::length(dv) - constraint->rest_length;
	
	double F = drest_length * constraint->k;
	//printf("Fpos: %f", constraint->u->pos.x);
	glm::dvec3 F_u = F * (constraint->v->pos - constraint->u->pos) / (glm::length((constraint->v->pos - constraint->u->pos)));
	constraint->u->force += F_u;
	constraint->v->force -= F_u;
}


void updateRestLength(SpringConstraint* constraint, float time) {
	if (constraint->constraint != NULL) {
		//printf("%f\n", constraint->constraint->eval(time));
		constraint->rest_length = constraint->constraint->eval(time);
	}
	
}

HardConstraint::HardConstraint(Particle * u, Particle * v, double rest_length, Constraint::Constraint* constraint)
	: u(u), v(v), rest_length(rest_length), constraint(constraint) {};

void applyHardConstraint(HardConstraint* constraint) {
	Particle *a = constraint->u;
	Particle *b = constraint->v;
	glm::dvec3 delta = b->pos - a->pos;
	double deltalength = glm::sqrt(glm::dot(delta, delta));
	double diff = (deltalength - constraint->rest_length);
	a->pos += (delta / deltalength) * 0.5 * diff;
	b->pos -= (delta / deltalength) * 0.5 * diff;
}

void updateHardConstraints(HardConstraint* constraint, float time) {
	if (constraint->constraint != NULL) {
		//printf("%f\n", constraint->constraint->eval(time));
		constraint->rest_length = constraint->constraint->eval(time);
	}
}


namespace Constraint {
	SinConstraint::SinConstraint(float amplitude, float frequency, float phase_offset, float base_length)
		: amplitude(amplitude), frequency(frequency), phase_offset(phase_offset), base_length(base_length) {};
	float SinConstraint::eval(float time) {
		return amplitude * sin(2 * M_PI * frequency * time + phase_offset) + base_length;
	}
}


//change to floating point parameter input (seconds)
//double Constraint::sinConstraint(int x) {
//	//return exp((x % 1000) / 1000.0);
//	return ((sin(x / 1000.0) * .5 + 1.5) + .2);
//}
//
//double Constraint::sinConstraint0(int x) {
//	//return exp((x % 1000) / 1000.0);
//	x += M_PI * 500;
//	return ((sin(x/ 1000.0) * .5 + 1.5) + .2);
//}
//
//double Constraint::sinConstraint1(int x) {
//	x += M_PI * 200;
//	return ((sin(x / 1000.0) * .5 + .5) + .2);
//}