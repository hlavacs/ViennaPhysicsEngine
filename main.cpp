
#define _USE_MATH_DEFINES
#include <cmath>

#include <stdio.h>
#include <array>
#include <set>
#include <vector>
#include <algorithm>
#include <iterator>



#include "GJK.h"

int main() {
	//Set up level geometry

	/* BBox ground;
	ground.pos = vec3( 0.0, 0, 0.0);
	ground.min = vec3(-500, -500,-500);
	ground.max = vec3( 500, 0, 500);
	ground.matRS = mat3(1.0);
	ground.matRS_inverse = inverse(mat4(1.0));


	BBox box;
	box.pos = vec3( 0.0,  0.3, 0.0);
	box.min = vec3(-0.5, -0.5, -0.5);
	box.max = vec3( 0.5,  0.5,  0.5);
	box.matRS = mat3(1.0);
	box.matRS_inverse = inverse(mat4(1.0));

	vec3 mtv(0,0,0); //minimum translation vector

	auto hit = gjk(&box, &ground, &mtv);

	Polytope point;
	point.point( vec3(0.0f, -0.1f, 0.0f), mat3(1.0));

	vec3 pointpos(0.0f, 0.0f, 0.0f);
	Vertex point_n[] = { {{1}, {}} };
	point.num_points = 1;
	point.points2 = (float*)&pointpos;
	point.pos = vec3(0.0f, -0.1f, 0.0f);
	point.matRS = mat3(1.0);
	point.matRS_inverse = inverse(mat4(1.0));
	point.vertices = point_n;

	auto hit2 = gjk(&point, &ground, &mtv);

	Polytope lineseg;
	vec3 linepoints[] = { vec3(0.0f, 0.0f, 0.0f), vec3(1.0f, 1.0f, 1.0f) };
	Vertex lineseg_n[] = 	{ 	  { {0} } //every vertex is member of 1 edge
								, { {0} } 
							};
	Edge lineseg_e[] = { {0,1} }; 	//only one edge
	Face lineseg_f[] = { {} };		//no face

	lineseg.num_points = 2;
	lineseg.points = (float*)&linepoints;
	lineseg.pos = vec3(0.0f, -0.2f, 0.0f);
	lineseg.matRS = mat3(1.0);
	lineseg.matRS_inverse = inverse(mat4(1.0));
	lineseg.vertices = lineseg_n;
	lineseg.edges = lineseg_e;
	lineseg.faces = lineseg_f;

	auto hit3 = gjk(&lineseg, &ground, &mtv);


	Polytope triangle;
	vec3 tripoints[] = { vec3(0.0f, 0.0f, -0.0f), vec3(1.0f, 1.0f, 0.0f), vec3(-1.0f, 1.0f, 0.0f) };
	Vertex triangle_n[] = 	{ 		{ {0,2}, {0} } //every vertex is member of 2 edges and 1 face
								, 	{ {0,1}, {0} }
								, 	{ {1,2}, {0} } 
							};
	Edge triangle_e[] = { {0,1}, {1,2}, {2,0} }; 	//three edges
	Face triangle_f[] = { {{0,1,2}} };			//one face with 3 vertices

	triangle.num_points = 3;
	triangle.points = (float*)&tripoints;
	triangle.pos = vec3(-0.52f, -0.3f, 0.2111f);
	triangle.matRS = mat3(1.0);
	triangle.matRS_inverse = inverse(mat4(1.0));
	triangle.vertices = triangle_n;
	triangle.edges = triangle_e;
	triangle.faces = triangle_f;

	auto hit4 = gjk(&triangle, &ground, &mtv);

	Polytope quad;
	vec3 quadpoints[] = { vec3(0.0f, 0.0f, -0.0f), vec3(0.0f, 0.0f, 1.0f), vec3(0.0f, 1.0f, 1.0f), vec3(0.0f, 1.0f, 0.0f)  };
	Vertex quad_n[] = 	{ 		{ {0,3}, {0} } //every vertex is member of 2 edges and 1 face
							,	{ {0,1}, {0} } 
							, 	{ {1,2}, {0} }
							, 	{ {2,3}, {0} }
						};
	Edge quad_e[] = { {0,1}, {1,2}, {2,3}, {3,0} }; 	//four edges
	Face quad_f[] = { {{0,1,2,3}} };				//one face with 4 vertices

	quad.num_points = 4;
	quad.points = (float*)&quadpoints;
	quad.pos = vec3(-0.0f, -0.33f, 0.0f);
	quad.matRS = mat3(1.0);
	quad.matRS_inverse = inverse(mat4(1.0));
	quad.vertices = quad_n;
	quad.edges = quad_e;
	quad.faces = quad_f;

	auto hit5 = gjk(&triangle, &quad, &mtv);

	auto hit6 = gjk(&quad, &ground, &mtv);


	//remove interpenetration between triangle and ground, mtv is our minimal translation vector for the quad
	quad.pos += mtv;
	vec3 mtv1;
	auto hit7 = gjk(&quad, &ground, &mtv1); //check that both still touch, but there is no more interpenetration

	vec3 support = quad.support( -1.0f * mtv ); //this is a point where the quad and the ground touch

	//get all vertex neighbors of vertex 0
	int v = 0;
	std::set<int> neighbors;
	get_neighbors_of_vertex(quad, v, neighbors);
	//neighbors now contains the direct neighbors of vertex 0, i.e. 1 and 3

	//get all edges of face 0
	int f = 0;
	std::set<int> edges;
	get_edges_of_face( quad, f, edges);
	//edges now contains all edges of face 0, i.e. 0, 1, 2, 3

	//get the faces that contain a given edge
	int e = 0;
	std::set<int> faces;
	get_faces_of_edge( quad, e, faces);
	//contains 0

	//get the neighboring faces of a given face
	std::set<int> faces2;
	get_neighbors_of_face( quad, f, faces2);
	//empty since there is only one face
	*/

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
	auto &edges = box.get_edges_of_face( f );

	//get the faces that contain a given edge
	int e = 0;
	std::set<int> faces;
	box.get_faces_of_edge( e, faces);

	//get the neighboring faces of a given face
	std::set<int> faces2;
	box.get_neighbors_of_face( f, faces2);

}
