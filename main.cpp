
#define _USE_MATH_DEFINES
#include <cmath>

#include <stdio.h>
#include <array>
#include <set>
#include <vector>
#include <algorithm>
#include <iterator>

#include "sat.h"
#include "GJK.h"
#include "contact.h"

int main() {

	Line3D line{ {0.f,0.f, -1.f}, {0.f,0.0f,1.f} };

	Tetrahedron tet{ {-1.0f, 0.0f, -1.0f}, {1.0f, 0.0f, -1.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 1.0f, 0.5f} };
	Triangle3D tri{ {-1.0f, 0.0f, -1.0f}, {1.0f, 0.0f, -1.0f}, {0.0f, 0.0f, 1.0f} };

	Box ground{ {0.0f, -50.0f, 0.0f}, scale( mat4(1.0f), vec3(100.0f, 100.0f, 100.0f)) };
	Box box{ {0.0f, 0.41f, 0.0f} };
	vec3 mtv(0,1,0); //minimum translation vector
	auto hit1 = gjk(&box, &ground, &mtv);

	box.pos += mtv;
	std::set<contact> ct;
	contacts( &box, &ground, &mtv, ct);

	
	auto n0 = box.get_face_normal(0);
	auto n1 = box.get_face_normal(1);
	auto n2 = box.get_face_normal(2);
	auto n3 = box.get_face_normal(3);
	auto n4 = box.get_face_normal(4);
	auto n5 = box.get_face_normal(5);

	auto hit2 = sat( (Collider*)&box, (Collider*)&ground, &mtv);

	Point3D point{ {0,-10,0} };
	auto hit3 = gjk(&point, &ground, &mtv);

	Triangle3D triangle{ {-1,0,0}, {1,0,0}, {0,1,0} };
	auto hit4 = gjk(&triangle, &ground, &mtv);

	Quad3D quad{ {-1,0,-1}, {1,0,-1}, {1,0,1}, {-1,0,1} };
	auto hit5 = gjk(&quad, &ground, &mtv);

	auto hit7 = gjk(&triangle, &quad, &mtv);

	//--------------------------------------------------------------------

	//get all vertex neighbors of vertex 0
	int v = 0;
	const std::vector<int>& neighbors = box.get_vertex_neighbors( v );

	int f = 0;
	const std::vector<int>& faces2 = box.get_face_neighbors(f);



}
