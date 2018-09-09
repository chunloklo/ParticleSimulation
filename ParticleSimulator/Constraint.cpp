#include "Constraint.h"
#include <glm/glm.hpp>

//either- between particles, hard environmental constraints, or

SpringConstraint::SpringConstraint(Particle * u, Particle * v, double k, double rest_length)
	: u(u), v(v), k(k), rest_length(rest_length), rest_length_orig(rest_length), restLengthModifier(NULL) {};

SpringConstraint* SpringConstraint::SpringConstraintWithModifier(Particle* u, Particle* v, double k, double rest_length, double(*restLengthModifier) (int)) {
	SpringConstraint* constraint = new SpringConstraint(u, v, k, rest_length);
	constraint->restLengthModifier = restLengthModifier;
	return constraint;
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


void updateRestLength(SpringConstraint* constraint, double time) {
	if (constraint->restLengthModifier != NULL) {
		constraint->rest_length = constraint->rest_length_orig * constraint->restLengthModifier(time);
	}
	
}

HardConstraint::HardConstraint(Particle * u, Particle * v, double rest_length)
	: u(u), v(v), rest_length(rest_length) {};

void applyHardConstraint(HardConstraint* constraint) {
	Particle *a = constraint->u;
	Particle *b = constraint->v;
	glm::dvec3 delta = b->pos - a->pos;
	double deltalength = glm::sqrt(glm::dot(delta, delta));
	double diff = (deltalength - constraint->rest_length);
	a->pos += (delta / deltalength) * 0.5 * diff;
	b->pos -= (delta / deltalength) * 0.5 * diff;
}

double Constraint::sinConstraint(int x) {
	return ((sin(x / 1000.0) * .5 + .5) + .2);
}