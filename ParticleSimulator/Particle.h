#ifndef PARTICLE_H
#define PARTICLE_H

#include <glm/glm.hpp>

class Particle {
public:
	glm::dvec3 pos;
	glm::dvec3 pos_prev;
	glm::dvec3 velocity;
	double mass;

	glm::dvec3 force;

	//constructors
	Particle(glm::dvec3 position_prev, glm::dvec3 position);
	Particle();

	//Only valid for Euler forward integration. Use the default constructor for verlet Integration
	static Particle* ParticleVel(glm::dvec3 position, glm::dvec3 velocity);
};


void verletIntegration(Particle &particle, double delta_t, glm::dvec3 force);

void eulerForwardIntegration(Particle &particle, double delta_t);

#endif
