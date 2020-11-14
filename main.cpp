
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

int main() {

	Box ground( vec3( 0.0f, -50.0f, 0.0f), scale( mat4(1.0f), vec3(100.0f, 100.0f, 100.0f)) );
	Box box( vec3( 10.0f, -1.0f, 10.0f ) ) ;
	vec3 mtv(0,0,0); //minimum translation vector
	auto hit = gjk(&box, &ground, &mtv);

	Point point( vec3(0,-10,0) );
	hit = gjk(&point, &ground, &mtv);

	Triangle triangle( vec3(0.0, -0.0, 0.0), rotate( mat4(1.0f), (float)M_PI_2, vec3(1.0f, 0.0f, 0.0f) ) );
	hit = gjk(&triangle, &ground, &mtv);

	Quad quad( vec3(0.0, -0.01, 0.0) );
	hit = gjk(&quad, &ground, &mtv);

	hit = gjk(&triangle, &quad, &mtv);

	//--------------------------------------------------------------------

	//get all vertex neighbors of vertex 0
	int v = 0;
	std::set<int> neighbors;
	box.get_neighbors_of_vertex( v, neighbors);

	//get all edges of face 0
	int f = 0;
	auto edges  = box.get_edges_of_face( f );
	auto &edges2 = box.get_edges_of_face( f );

	//get the faces that contain a given edge
	int e = 0;
	std::set<int> faces;
	box.get_faces_of_edge( e, faces);

	//get the neighboring faces of a given face
	std::set<int> faces2;
	box.get_neighbors_of_face( f, faces2);

}
