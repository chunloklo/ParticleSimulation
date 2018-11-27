#pragma once
#include <glm/glm.hpp>

using namespace glm;
struct Point {
	dvec2 pos;
	dvec2 vel;
};

struct Element {
	int i;
	int j;
	int k;
	double mu;
	double lambda;
};