#pragma once
//#include "GameMaths.h"

//Kevin's simple collider objects for collision detection
//Different shapes which inherit from Collider and implement
//support() function for use in GJK

using namespace glm;

//Base struct for all collision shapes
struct Collider {
    vec3    pos;            //origin in world space
    mat3    matRS;          //rotation/scale component of model matrix
    mat3    matRS_inverse; 

    Collider( vec3 pos, mat3 matRS ) {
        this->pos = pos;
        this->matRS = matRS;
        matRS_inverse = inverse( matRS );
    };

    virtual vec3 support(vec3 dir) = 0;
};

//BBox: AABB + Orientation matrix
struct BBox : Collider {
    vec3 min, max; //Assume these are axis aligned!

    vec3 support(vec3 dir){
        dir = matRS_inverse*dir; //find support in model space

        vec3 result;
        result.x = (dir.x>0) ? max.x : min.x;
        result.y = (dir.y>0) ? max.y : min.y;
        result.z = (dir.z>0) ? max.z : min.z;

        return matRS*result + pos; //convert support to world space
    }
};

//Sphere: NB Does not use RS matrix, scale the radius directly!
struct Sphere : Collider {
    float r;

    vec3 support(vec3 dir){
        return normalize(dir)*r + pos;
    }
};

//Cylinder: Height-aligned with y-axis (rotate using matRS)
struct Cylinder : Collider {
    float r, y_base, y_cap;

    vec3 support(vec3 dir){
        dir = matRS_inverse*dir; //find support in model space

        vec3 dir_xz = vec3(dir.x, 0, dir.z);
        vec3 result = normalize(dir_xz)*r;
        result.y = (dir.y>0) ? y_cap : y_base;

        return matRS*result + pos; //convert support to world space
    }
};

//Capsule: Height-aligned with y-axis
struct Capsule : Collider {
    float r, y_base, y_cap;

    vec3 support(vec3 dir){
        dir = matRS_inverse*dir; //find support in model space

        vec3 result = normalize(dir)*r;
        result.y += (dir.y>0) ? y_cap : y_base;

        return matRS*result + pos; //convert support to world space
    }
};

//Triangle: Kind of a hack 
// "All physics code is an awful hack" - Will, #HandmadeDev
//Need to fake a prism for GJK to converge
//NB: Currently using world-space points, ignore matRS and pos from base class
//Don't use EPA with this! Might resolve collision along any one of prism's faces
//Only resolve around triangle normal
struct TriangleCollider : Collider {
    vec3 points[3];
    vec3 normal;

    vec3 support(vec3 dir){
        //Find which triangle vertex is furthest along dir
        float dot0 = dot(points[0], dir);
        float dot1 = dot(points[1], dir);
        float dot2 = dot(points[2], dir);
        vec3 furthest_point = points[0];
        if(dot1>dot0){
            furthest_point = points[1];
            if(dot2>dot1) 
                furthest_point = points[2];
        }
        else if(dot2>dot0) {
            furthest_point = points[2];
        }

        //fake some depth behind triangle so we have volume
        if(dot(dir, normal)<0) furthest_point-= normal; 

        return furthest_point;
    }
};


constexpr int MAX_NEIGHBORS = 8;   //adapt the max if you need more neighbors
struct Vertex {
    std::vector<int> edges;       //indices of all edges of this vertex
    std::vector<int> faces;       //indices of all faces of this vertex
};

struct Edge {
    std::array<int, 2>  vertices;  //indices of the two vertices of this edge

    bool contains_vertex( int v ) {
        return vertices[0] == v || vertices[1] == v;
    }
};

struct Polytope; 

struct Face {
    std::vector<int> edges;         //indices of all edges of this face

    bool contains_edge( int v ) {
        auto result = std::find( edges.begin(), edges.end(), v);
        return result != edges.end();
    }

    bool contains_vertex( Polytope &polytope, int v );
};

//Polytope: Just a set of points
struct Polytope : Collider {
    std::vector<vec3>   points2;
    std::vector<Vertex> vertices2;
    std::vector<Edge>   edges2;
    std::vector<Face>   faces2;

    Polytope( vec3 pos, mat3 matRS ) : Collider(pos, matRS) {}

    //Dumb O(n) support function, just brute force check all points
    vec3 support(vec3 dir){
        dir = matRS_inverse*dir;

        vec3 furthest_point = points2[0]; 
        float max_dot = dot(furthest_point, dir);

        if( true ) {           //if( neighbors == nullptr ) {  //replace the if statement so this branch is only taken if no neighbor info available
            std::for_each(  std::begin(points2), std::end(points2), 
                            [&]( auto &v) {
                                float d = dot(v, dir);
                                if(d>max_dot){
                                    max_dot = d;
                                    furthest_point = v;
                                }
                            } );
        } else {
            //ADD YOUR CODE HERE TO ITERATE THROUGH NEIGHBORS rather than iterate through all points
        }

        vec3 result = matRS*furthest_point + pos; //convert support to world space
        return result;
    }

    void get_neighbors_of_vertex(int v, std::set<int> &neighbors);
    std::vector<int>& get_edges_of_vertex( int v );
    std::vector<int>& get_edges_of_face( int f );
    void get_faces_of_edge( int e, std::set<int> &faces);
    void get_neighbors_of_face( int f, std::set<int> &faces );

};

struct Point : Polytope {
    Point( vec3 pos = vec3(0.0f, 0.0f, 0.0f), mat3 matRS = mat3(1.0f) ) : Polytope(pos, matRS) {
        points2 = { vec3(0.0f, 0.0f, 0.0f) };
        vertices2 = { {} };
    };
};

struct Line : Polytope {
    Line( vec3 pos = vec3(0.0f, 0.0f, 0.0f), mat3 matRS = mat3(1.0f) ) : Polytope(pos, matRS ) {
        points2 = { vec3{-0.5f, 0.0f, 0.0f}, vec3{0.5f, 0.0f, 0.0f} };
        vertices2 = { { { 0 } }, { { 0 } } }; 
        edges2 = { {{0,1}} };
    };
};

struct Triangle : Polytope {
    Triangle( vec3 pos = vec3(0.0f, 0.0f, 0.0f), mat3 matRS = mat3(1.0f) )  : Polytope(pos, matRS ) {
	    points2 = { vec3(-0.5f, 0.0f, 0.0f), vec3(0.5f, 0.0f, 0.0f), vec3(0.0f, 0.0f, 0.5f) };
	    vertices2 = 	{ 	    { {0,2}, {0} } //every vertex is member of 2 edges and 1 face
							, 	{ {0,1}, {0} }
							, 	{ {1,2}, {0} } 
						};
	    edges2 = { {{0,1}}, {{1,2}}, {{2,0}} }; 	//3 edges
	    faces2 = { {{0,1,2}} };			            //one face with 3 edges
    };
};

struct Quad : Polytope {
    Quad( vec3 pos = vec3(0.0f, 0.0f, 0.0f), mat3 matRS = mat3(1.0f) )  : Polytope(pos, matRS ) {
	    points2 = { vec3(-0.5f, 0.0f, -0.5f), vec3(0.5f, 0.0f, -0.5f), vec3(0.5f, 0.0f, 0.5f), vec3(-0.5f, 0.0f, 0.5f)  };
	    vertices2 = 	{ 	    { {0,3}, {0} } //every vertex is member of 2 edges and 1 face
							, 	{ {0,1}, {0} }
							, 	{ {1,2}, {0} } 
							, 	{ {2,3}, {0} } 
						};
	    edges2 = { {{0,1}}, {{1,2}}, {{2,3}}, {{3,0}} }; 	//4 edges
	    faces2 = { {{0,1,2,3}} };			    //one face with 4 edges
    };
};

struct Box : Polytope {
    Box( vec3 pos = vec3(0.0f, 0.0f, 0.0f), mat3 matRS = mat3(1.0f) )  : Polytope(pos, matRS ) {
	    points2 = { vec3(-0.5f, -0.5f, -0.5f), vec3(0.5f, -0.5f, -0.5f), vec3(0.5f, -0.5f, 0.5f), vec3(-0.5f, -0.5f, 0.5f),
                    vec3(-0.5f,  0.5f, -0.5f), vec3(0.5f,  0.5f, -0.5f), vec3(0.5f,  0.5f, 0.5f), vec3(-0.5f,  0.5f, 0.5f)};
	    vertices2 = 	{ 	                            //every vertex is member of 3 edges and 3 faces
                                { {0,3, 8}, {0,2,5} }   // 0  
							, 	{ {0,1, 9}, {0,2,3} }   // 1
							, 	{ {1,2,10}, {0,3,4} }   // 2
							, 	{ {2,3,11}, {0,4,5} }   // 3
                            ,   { {4,7, 8}, {1,2,5} }   // 4
							, 	{ {4,5, 9}, {1,2,3} }   // 5
							, 	{ {5,6,10}, {1,3,4} }   // 6
							, 	{ {6,7,11}, {1,4,5} }   // 7
						};
        //                 0        1        2        3
	    edges2 = {      {{0,1}}, {{1,2}}, {{2,3}}, {{3,0}}
        //                 4        5        6        7
                    ,   {{4,5}}, {{5,6}}, {{6,7}}, {{7,4}}
        //                 8        9       10       11
                    ,   {{0,4}}, {{1,5}}, {{2,6}}, {{3,7}}  }; 	//12 edges

        //                   0              1
	    faces2 = {      {{0,1,2,3}},   {{4,5,6,7}}
        //                   2              3
                    ,   {{0,9,4,8}},   {{1,10,5,9}}
        //                    4              5
                    ,   {{2,11,6,10}}, {{3,8,7,11}} };		    //6 faces, each having 4 edges
    };
};


bool Face::contains_vertex( Polytope &polytope, int v ) {
    auto it = std::find_if( std::begin(edges), std::end(edges), 
                    [&]( auto &e){
                       return polytope.edges2[e].contains_vertex(v);
                    });
    return it != std::end(edges);
}


//Polytope functions

//Return all neighboring vertices of a given vertex
void Polytope::get_neighbors_of_vertex(int v, std::set<int> &neighbors) {
	Vertex &vertex = vertices2[v];	                    //edge info of vertex v
	std::for_each( std::begin(vertex.edges) , std::end(vertex.edges), //go through all edges that contain v
					[&]( auto &e ){ 
						auto &i = edges2[e].vertices;       //access the vertices of the edge
						if(i[0]!=v) neighbors.insert(i[0]); //do not add the vertex itself, only true neighbor
						if(i[1]!=v) neighbors.insert(i[1]); 
					} );
}

//return all edges a given vertex is part of
std::vector<int>& Polytope::get_edges_of_vertex( int v ) {
	Vertex &vertex = vertices2[v];  		        //membership info of this vertex
	return vertex.edges;
}

//return all edges of a given face
std::vector<int>& Polytope::get_edges_of_face( int f ) {
	Face &face = faces2[f];                         //information of face f
	return face.edges;
}

//return zero, one or two faces that contain a given edge
void Polytope::get_faces_of_edge( int e, std::set<int> &faces) {
    for( int f=0; f<faces2.size(); ++f) {
        Face &face = faces2[f];
        if( face.contains_edge(e) )     //it contains the edge
            faces.insert(f);
    }
}

//return all neighboring faces of a given face
void Polytope::get_neighbors_of_face( int f, std::set<int> &faces ) {
    auto& edges = faces2[f].edges;        //get all edges of the face
    std::for_each( std::begin(edges), std::end(edges), 	    //go through all edges and get their faces
					[&](auto &e) { 
                        get_faces_of_edge( e, faces );
                    } ); 
    faces.erase(f);         //remove the given face since it is not its own neighbor
}

