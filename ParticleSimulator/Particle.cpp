//Jeff Chastine
#include <Windows.h>
#include <GL\glew.h>
#include <GL\freeglut.h>
#include <iostream>
#include <glm/glm.hpp>

#include "Particle.h"


Particle::Particle(glm::dvec3 pos_prev, glm::dvec3 pos) : pos_prev(pos_prev), pos(pos), velocity(pos - pos_prev), mass(1.0), force(0.0) {};	

//Particle::Particle(glm::dvec3 pos, glm::dvec3 velocity) : pos_prev(pos), pos(pos), velocity(velocity) {};

Particle::Particle() : pos_prev(0), pos(0), velocity(0), mass(1.0), force(0.0) {};

void verletIntegration(Particle &particle, double delta_t, glm::dvec3 acceleration) {
	glm::dvec3 pos = particle.pos;
	particle.pos = particle.pos + (particle.pos - particle.pos_prev) * delta_t + acceleration * delta_t * delta_t;
	particle.pos_prev = pos;
}

void eulerForwardIntegration(Particle &particle, double delta_t) {
	glm::dvec3 accel = particle.force / particle.mass;
	particle.pos_prev = particle.pos;
	particle.velocity = particle.velocity + accel * delta_t;
	particle.pos = particle.pos + particle.velocity * delta_t;
}

Particle* Particle::ParticleVel(glm::dvec3 position, glm::dvec3 velocity) {
	Particle* p = new Particle();
	p->pos = position;
	p->velocity = velocity;
	return p;
}
