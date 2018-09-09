//Jeff Chastine
#include <Windows.h>
#include <GL\glew.h>
#include <GL\freeglut.h>
#include <iostream>
#include <algorithm>

#include "Particle.h"
#include "Util.h"
#include "Constraint.h"

#include <vector>

using namespace std;

void resetForces(vector<Particle*>& particles) {
	for (int i = 0; i < particles.size(); i++) {
		particles[i]->force.x = 0;
		particles[i]->force.y = 0;
		particles[i]->force.z = 0;
	}
}


void changeViewPort(int w, int h)
{
	// Prevent a divide by zero, when window is too short
	// (you cant make a window of zero width).
	if (h == 0)
		h = 1;
	float ratio = (float) w / h;

	// Use the Projection Matrix
	glMatrixMode(GL_PROJECTION);

	// Reset Matrix
	glLoadIdentity();

	// Set the viewport to be the entire window
	glViewport(0, 0, w, h);

	// Set the correct perspective.
	gluPerspective(90, ratio, 1, 100000000);
	gluLookAt(0.0f, 0.0f, 10.0f,
		0.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f);
	// Get Back to the Modelview
	glMatrixMode(GL_MODELVIEW);
}





float angle = 0.0f;

int frame = 0;
Particle *a;

double timeSinceStart;
double oldTimeSinceStart;

vector<Particle*> particles;
vector<SpringConstraint*> springConstraints;
vector<HardConstraint*> hardConstraints;

void startup()
{
	particles.reserve(10);
	timeSinceStart = glutGet(GLUT_ELAPSED_TIME);

	oldTimeSinceStart = timeSinceStart;
	/*
	glm::dvec3 pos = glm::dvec3(0, 0, -2);
	glm::dvec3 prev_pos = glm::dvec3(0, 0, -2);
	a = new Particle(prev_pos, pos);
	//a->velocity = glm::dvec3(30, 3, 0);
	*/
	Particle* temp;
	temp = Particle::ParticleVel(glm::dvec3(-1.01, 1, -2), glm::dvec3(-100, 0, 0));
	particles.push_back(temp);

	temp = Particle::ParticleVel(glm::dvec3(1, 1, -2), glm::dvec3(0, 0, 0));
	particles.push_back(temp);

	temp = Particle::ParticleVel(glm::dvec3(1, -1, -2), glm::dvec3(0, 0, 0));
	particles.push_back(temp);

	temp = Particle::ParticleVel(glm::dvec3(-1, -1, -2), glm::dvec3(0, 0, 0));
	particles.push_back(temp);

	//hard constraint particles
	temp = Particle::ParticleVel(glm::dvec3(-5, 5, -2), glm::dvec3(0, 0, 0));
	particles.push_back(temp);

	temp = Particle::ParticleVel(glm::dvec3(-4, 4, -2), glm::dvec3(0, 0, 0));
	particles.push_back(temp);

	//spring constraints
	springConstraints.push_back(new SpringConstraint(particles[0], particles[1], 1000, 1));
	springConstraints.push_back(new SpringConstraint(particles[1], particles[2], 500, 1));
	springConstraints.push_back(new SpringConstraint(particles[2], particles[3], 500, 1));
	springConstraints.push_back(new SpringConstraint(particles[3], particles[0], 500, 1));
	springConstraints.push_back(new SpringConstraint(particles[0], particles[2], 500, 1.414));

	springConstraints[0]->restLengthModifier = Constraint::sinConstraint;

	hardConstraints.push_back(new HardConstraint(particles[4], particles[5], 1));

	//6
	temp = Particle::ParticleVel(glm::dvec3(0, -2, -2), glm::dvec3(0, 0, 0)); particles.push_back(temp);
	//7
	temp = Particle::ParticleVel(glm::dvec3(1, -2, -2), glm::dvec3(0, 0, 0)); particles.push_back(temp);
	//8
	temp = Particle::ParticleVel(glm::dvec3(.5, -2 + 1.732050 / 2, -2), glm::dvec3(0, 0, 0)); particles.push_back(temp);

	springConstraints.push_back(SpringConstraint::SpringConstraintWithModifier(particles[6], particles[7], 1000, 1, NULL));
	springConstraints.push_back(SpringConstraint::SpringConstraintWithModifier(particles[7], particles[8], 1000, 1, NULL));
	springConstraints.push_back(SpringConstraint::SpringConstraintWithModifier(particles[8], particles[6], 1000, 1, Constraint::sinConstraint));



}



double dampening_factor = -3;
double friction_factor = -100;
double epsilon = 0.01;


void simulate()
{
	double delta_t;
	delta_t = .0005;
	
	timeSinceStart = glutGet(GLUT_ELAPSED_TIME);
	//double delta_t = timeSinceStart - oldTimeSinceStart;
	//oldTimeSinceStart = timeSinceStart;
	//delta_t /= 1000;
	

	//gravity
	for (int i = 0; i < particles.size(); i++) {
		particles[i]->force.y = -9.8 * particles[i]->mass;
	}

	//update spring rest_lengths
	for (int i = 0; i < springConstraints.size(); i++) {
		updateRestLength(springConstraints[i], timeSinceStart);
	}

	//spring force
	for (int i = 0; i < springConstraints.size(); i++) {
		applySpringForce(springConstraints[i]);
	}

	//viscous damping
	for (int i = 0; i < particles.size(); i++) {
		Particle *particle = particles[i];
		particle->force += (particle->velocity * dampening_factor);
		//from 1 - 10
	}

	//friction
	for (int i = 0; i < particles.size(); i++) {
		Particle *particle = particles[i];
		if (particle->pos.y - epsilon <= -2) {
			particle->force += (particle->velocity * friction_factor);
		}
		
		//from 1 - 10
	}


	//integration
	for (int i = 0; i < particles.size(); i++) {
		eulerForwardIntegration(*particles[i], delta_t);
	}

	for (int i = 0; i < 4; i++) {
		//box constraint
		//It is currently vec(8,8,8) and vec (-8,-8,-8);
		for (int i = 0; i < particles.size(); i++) {
			Particle *particle = particles[i];
			//bottom and left
			glm::dvec3 posNew = elementwiseMax(glm::dvec3(-8, -2, -8), particle->pos);

			//top and right
			posNew = elementwiseMin(glm::dvec3(8, 8, 8), posNew);

			if (particle->pos != posNew) {
				particle->pos = posNew;
				particle->velocity = (particle->pos - particle->pos_prev) / delta_t;
			}
		}

		for (int j = 0; j < hardConstraints.size(); j++) {
			applyHardConstraint(hardConstraints[j]);
		}
	}
	

	resetForces(particles);
}


void render()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	// Reset transformations
	glLoadIdentity();
	// Set the camera
	if (frame == 0) {
		startup();
	}

	simulate();
	drawParticles(particles);
	drawSpringConstraints(springConstraints);
	drawHardConstraints(hardConstraints);

	
	glutSwapBuffers();
	frame++;
}

void keyPressed(unsigned char key, int x, int y)
{
	switch (key) {
	case 'a':
		springConstraints[0]->rest_length -= 0.1;
		break;
	case 'd':
		springConstraints[0]->rest_length += 0.1;
		break;
	}
	//printf("%f\n", springConstraints[0]->rest_length);
}

int main(int argc, char* argv[]) {

	// Initialize GLUT
	glutInit(&argc, argv);
	// Set up some memory buffers for our display
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	// Set the window
	glutInitWindowPosition(100, 100);
	glutInitWindowSize(800, 800);
	// Create the window with the title "Hello,GL"
	glutCreateWindow("Particle Simulator View");
	// Bind the two functions (above) to respond when necessary
	glutReshapeFunc(changeViewPort);
	glutDisplayFunc(render);
	glutIdleFunc(render);

	glutKeyboardFunc(keyPressed);

	// Very important!  This initializes the entry points in the OpenGL driver so we can 
	// call all the functions in the API.
	GLenum err = glewInit();
	if (GLEW_OK != err) {
		fprintf(stderr, "GLEW error");
		return 1;
	}

	glutMainLoop();
	return 0;
}
