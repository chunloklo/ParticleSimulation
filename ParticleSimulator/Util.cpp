#include <Windows.h>
#include <GL\glew.h>
#include <GL\freeglut.h>
#include <iostream>
#include <algorithm>
#include "Particle.h"
#include <vector>

# define M_PI           (3.14159265358979323846)  /* pi */

static int sides = 10;

//render functions
void drawCircle(double x, double y, double z, float radius) {
	glPushMatrix();
	glTranslated(x, y, z);

	glBegin(GL_TRIANGLE_FAN);
	for (int i = 0; i < sides; i++) {
		glPushMatrix();
		glVertex3f(cos((float)i / sides * 2 * M_PI) * radius, sin((float)i / sides * 2 * M_PI) * radius, -z);
		glPopMatrix();
	}
	glEnd();

	glPopMatrix();
}

void drawSphere(double x, double y, double z, float radius) {
	glPushMatrix();
	glTranslated(x, y, z);
	glutSolidSphere(radius, 50, 50);
	glPopMatrix();
}

void drawParticles(std::vector<Particle*>& particles) {
	for (int i = 0; i < particles.size(); i++) {
		drawSphere(particles[i]->pos.x, particles[i]->pos.y, particles[i]->pos.z, .1);
	}
}

glm::dvec3 elementwiseMax(glm::dvec3 u, glm::dvec3 v) {
	return glm::dvec3(std::max(u.x, v.x), std::max(u.y, v.y), std::max(u.z, v.z));
	//return glm::dvec3(1, 1, 1);
}

glm::dvec3 elementwiseMin(glm::dvec3 u, glm::dvec3 v) {
	return glm::dvec3(std::min(u.x, v.x), std::min(u.y, v.y), std::min(u.z, v.z));
}