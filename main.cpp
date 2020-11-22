
#define _USE_MATH_DEFINES
#include <cmath>

#include <stdio.h>
#include <array>
#include <set>
#include <vector>
#include <algorithm>
#include <iterator>

#include "gjk_epa.h"
#include "collision.h"
#include "contact.h"

int main() {
	Line3D line{ {0.f,0.f, -1.f}, {0.f,0.0f,1.f} };

	Tetrahedron tet( {-1.0f, 0.0f, -1.0f}, {1.0f, 0.0f, -1.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 1.0f, 0.5f}  );
	Tetrahedron tet2{};

	Triangle3D tri{ {-1.0f, 0.0f, -1.0f}, {1.0f, 0.0f, -1.0f}, {0.0f, 0.0f, 1.0f} };

	Box ground{ {0.0f, -50.0f, 0.0f}, scale( mat4(1.0f), vec3(100.0f, 100.0f, 100.0f)) };
	Box box{ {0.0f, 0.41f, 0.0f} };
	vec3 mtv(0,1,0); //minimum translation vector
	auto hit1 = gjk( box, ground, mtv);

	box.pos() += mtv;
	std::set<contact> ct;
	contacts( box, ground, mtv, ct);

	vec3 p0{0,1,2};
	vec3 p1{3,4,5};
	auto pp = pluecker_point( {7,8,9} );
	auto pl = pluecker_line( {7,8,9}, p1 );
	auto ppl = pluecker_plane( {7,8,9}, 3 );

	auto n0 = box.face(0).normalW();

	auto hit2 = collision( box, ground, mtv);

	Point3D point{ {0,-10,0} };
	auto hit3 = gjk( point, ground, mtv);

	Triangle3D triangle{ {-1,0,0}, {1,0,0}, {0,1,0} };
	auto hit4 = gjk( triangle, ground, mtv);

	Quad3D quad{ {-1,0,-1}, {1,0,-1}, {1,0,1}, {-1,0,1} };
	auto hit5 = gjk( quad, ground, mtv);

	auto hit7 = gjk( triangle, quad, mtv);





}
