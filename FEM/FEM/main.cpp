//Jeff Chastine
#include <Windows.h>
#include <GL\glew.h>
#include <GL\freeglut.h>
#include <iostream>
#include <algorithm>

#include <vector>

#include "Test.h"
#include "Point.h"
#include "Util.h"
#include "FEMProcedure.h"

using namespace std;

void changeViewPort(int w, int h)
{
	// Prevent a divide by zero, when window is too short
	// (you cant make a window of zero width).
	if (h == 0)
		h = 1;
	float ratio = (float)w / h;

	// Use the Projection Matrix
	glMatrixMode(GL_PROJECTION);

	// Reset Matrix
	glLoadIdentity();

	// Set the viewport to be the entire window
	glViewport(0, 0, w, h);

	// Set the correct perspective.
	gluPerspective(90, ratio, 1, 10000000);
	gluLookAt(0.0f, 0.0f, 5.0f,
		0.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f);
	// Get Back to the Modelview
	glMatrixMode(GL_MODELVIEW);
}

vector<Point> refVector;
vector<Point> pointVector;
vector<Element> eleVector;
vector<dmat2> referenceInv;
vector<double> undeformedVol;
vector<dvec2> forceVector;


void simulate()
{
	computeElasticForces(forceVector, pointVector, eleVector, referenceInv, undeformedVol);
}

void render()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Reset transformations
	glLoadIdentity();

	simulate();

	//draw points
	for (size_t i = 0; i < pointVector.size(); i++) {
		drawSphere(pointVector[i].pos.x, pointVector[i].pos.y, -1, .05);
	}
	glColor3f(0.0f, 1.0f, 0.0f);
	//draw points
	for (size_t i = 0; i < refVector.size(); i++) {
		drawSphere(refVector[i].pos.x, refVector[i].pos.y, -1, .05);
	}
	glColor3f(1.0f, 1.0f, 1.0f);

	//draw element
	for (size_t i = 0; i < eleVector.size(); i++) {
		Point p1 = pointVector[eleVector[i].i];
		Point p2 = pointVector[eleVector[i].j];
		Point p3 = pointVector[eleVector[i].k];

		dvec3 pos1 = dvec3(p1.pos.x, p1.pos.y, -1);
		dvec3 pos2 = dvec3(p2.pos.x, p2.pos.y, -1);
		dvec3 pos3 = dvec3(p3.pos.x, p3.pos.y, -1);

		drawLineEndpoints(pos1, pos2);
		drawLineEndpoints(pos2, pos3);
		drawLineEndpoints(pos3, pos1);
	}

	//draw forces
	glColor3f(1.0f, 1.0f, 0.0f);
	printf("%f, %f\n", forceVector[2].x, forceVector[2].y);
	for (size_t i = 0; i < forceVector.size(); i++) {
		dvec3 start = dvec3(pointVector[i].pos.x, pointVector[i].pos.y, -1);
		dvec3 end = start + dvec3(forceVector[i].x, forceVector[i].y, 0);
		drawLineEndpoints(start, end);
	}
	glColor3f(1.0f, 1.0f, 1.0f);

	glutSwapBuffers();
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

	// Very important!  This initializes the entry points in the OpenGL driver so we can 
	// call all the functions in the API.
	GLenum err = glewInit();
	if (GLEW_OK != err) {
		fprintf(stderr, "GLEW error");
		return 1;
	}

	readTestData(refVector, pointVector, eleVector);
	precomputation(refVector, eleVector, referenceInv, undeformedVol);
	for (size_t i = 0; i < pointVector.size(); i++) {
		forceVector.push_back(dvec2(0));
	}


	glutMainLoop();
	return 0;
}
