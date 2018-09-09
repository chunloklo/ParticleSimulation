#include <Windows.h>
#include <GL\glew.h>
#include <GL\freeglut.h>
#include <iostream>
#include <algorithm>
#include "Particle.h"
#include "Constraint.h"
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

static void drawCylinder(int n = 10, float h = 2.0, float r = 1.0) {
	/*
		Function drw_polygon:
		Arguments:
			n - number of sides
			arg - starting angle (not so important at all)
			mult - multiplying sides to incrase their length
			v - cylinder height
	*/

	double x = 0;
	double y = 0;
	double angle = 2 * M_PI / n;
	glBegin(GL_TRIANGLE_STRIP);
	for (int i = 0; i <= n; i++) {
		double tot_angle = angle * i;
		
		glVertex3f(cos(tot_angle) * r, sin(tot_angle) * r, 0.0);
		glVertex3f(cos(tot_angle) * r, sin(tot_angle) * r, h);
		
	}
	glEnd();
}


static void drawCylinderEndpoints(glm::dvec3 u, glm::dvec3 v, float r) {

	glPushMatrix();
	glm::dvec3 delta = v - u;
	glm::dvec3 delta_norm = delta / glm::length(delta);
	double angle = acos(glm::dot(delta_norm, glm::dvec3(0, 0, 1)));
	glm::dvec3 axis = glm::cross(glm::dvec3(0, 0, 1), delta_norm);
	glTranslatef(u.x, u.y, u.z);
	glRotatef(angle / (2 * M_PI) * 360, axis.x, axis.y, axis.z);
	drawCylinder(10, glm::length(delta), r);
	glPopMatrix();
}


void drawSpringConstraints(std::vector<SpringConstraint*> &springConstraints) {
	for (int i = 0; i < springConstraints.size(); i++) {
		drawCylinderEndpoints(springConstraints[i]->u->pos, springConstraints[i]->v->pos, 0.03);
	}
}

void drawHardConstraints(std::vector<HardConstraint*> &hardConstraints) {
	for (int i = 0; i < hardConstraints.size(); i++) {
		drawCylinderEndpoints(hardConstraints[i]->u->pos, hardConstraints[i]->v->pos, 0.03);
	}
}