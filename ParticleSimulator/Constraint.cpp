#include "Constraint.h"
//either- between particles, hard environmental constraints, or

SpringConstraint::SpringConstraint(Particle * u, Particle * v, double k, double rest_length)
	: u(u), v(v), k(k), rest_length(rest_length) {};

void applySpringForce(SpringConstraint* constraint) {
	glm::dvec3 dv = constraint->u->pos - constraint->v->pos;
	double drest_length = glm::length(dv) - constraint->rest_length;
	
	double F = drest_length * constraint->k;
	//printf("Fpos: %f", constraint->u->pos.x);
	glm::dvec3 F_u = F * (constraint->v->pos - constraint->u->pos) / (glm::length((constraint->v->pos - constraint->u->pos)));
	constraint->u->force += F_u;
	constraint->v->force -= F_u;
}