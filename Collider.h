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
    std::array<int, 2> vertices;  //indices of vertices of this edge
};

struct Face {
    std::vector<int> vertices;    //indices of all vertices of this face

    bool contains( int v ) {
        auto result = std::find( vertices.begin(), vertices.end(), v);
        return result != vertices.end();
    }
};

//Polytope: Just a set of points
struct Polytope : Collider {
	int     num_points;                //number of vertices in the polytope
	float   *points;                   //(x0 y0 z0 x1 y1 z1 etc)

    std::vector<vec3>   points2;
    std::vector<Vertex> vertices2;
    std::vector<Edge>   edges2;
    std::vector<Face>   faces2;

    Vertex  *vertices = nullptr;       //each vertex gets one entry here
    int     num_edges;
    Edge   *edges = nullptr;           //list of all edges of this polytope
    int     num_faces;
    Face   *faces = nullptr;           //list of all faces of this polytope

    //Dumb O(n) support function, just brute force check all points
    vec3 support(vec3 dir){
        dir = matRS_inverse*dir;

        vec3 furthest_point = vec3(points[0], points[1], points[2]);
        float max_dot = dot(furthest_point, dir);

        if( true ) {           //if( neighbors == nullptr ) {  //replace the if statement so this branch is only taken if no neighbor info available
            for(int i=3; i<num_points*3; i+=3){
                vec3 v = vec3(points[i], points[i+1], points[i+2]);
                float d = dot(v, dir);
                if(d>max_dot){
                    max_dot = d;
                    furthest_point = v;
                }
            }
        } else {
            //ADD YOUR CODE HERE TO ITERATE THROUGH NEIGHBORS rather than iterate through all points
        }

        vec3 result = matRS*furthest_point + pos; //convert support to world space
        return result;
    }

    void point( vec3 pos, mat3 matRS ) {
        this->pos = pos;
        this->matRS = matRS;
        matRS_inverse = inverse(matRS);
        num_points = 1;
        points2 = { vec3(0,0,0) };
        vertices2 = { {{}, {}} };
    };

    void line() {

    };

    void triangle() {

    };

    void quad() {

    };

    void box() {

    };

};



//Polytope functions

//Return all neighboring vertices of a given vertex
void get_neighbors_of_vertex(Polytope &polytope, int v, std::set<int> &neighbors) {
	Vertex &vertex = polytope.vertices[v];	                    //membership info of vertex v
	std::for_each( std::begin(vertex.edges) , std::end(vertex.edges), 
					[&]( auto &e ){ 
						auto &i = polytope.edges[e].vertices;
						if(i[0]!=v) neighbors.insert(i[0]); //do not add the vertex itself, only true neighbor
						if(i[1]!=v) neighbors.insert(i[1]); 
					} );
}

//return all edges a given vertex is part of
void get_edges_of_vertex( Polytope &polytope, int v, std::set<int> &edges ) {
	Vertex &vertex = polytope.vertices[v];  		        //membership info of this vertex
	std::for_each( std::begin(vertex.edges), std::end(vertex.edges), 	//go through all edges this vertex is member of
					[&](auto &e) { 
                        edges.insert(e); 
                    } ); 
}

//return all edges of a given face
void get_edges_of_face( Polytope &polytope, int f, std::set<int> &edges ) {
	Face &face = polytope.faces[f];                         //information of face f
	std::for_each( std::begin(face.vertices), std::end(face.vertices), 		//go through all vertices of this face
					[&]( auto &v) { 
                        get_edges_of_vertex( polytope, v, edges ); 
                    } );
}

//return zero, one or two faces that contain a given edge
void get_faces_of_edge( Polytope &polytope, int e, std::set<int> &faces) {
	Edge &edge = polytope.edges[e];  		               //info for this edge
	
    int v0 = edge.vertices[0];                             //vertex indices
    int v1 = edge.vertices[1];

    Vertex &vertex0 = polytope.vertices[v0];               //get both vertices of the edge
    Vertex &vertex1 = polytope.vertices[v1];

    std::set<int> tmp_faces;                               //get all faces the vertices belong to
    std::copy( std::begin(vertex0.faces), std::end(vertex0.faces), std::inserter( tmp_faces, std::begin(tmp_faces) ) );
    std::copy( std::begin(vertex1.faces), std::end(vertex1.faces), std::inserter( tmp_faces, std::begin(tmp_faces) ) );

    std::for_each( std::begin(tmp_faces), std::end(tmp_faces), 	    //go through all faces that contain the vertices
					[&](auto &f) { 
                        Face &face = polytope.faces[f];             //if face contains both vertices
                        if( face.contains(v0) && face.contains(v1)) //it contains the edge
                            faces.insert(f);
                    } ); 
}

//return all neighboring faces of a given face
void get_neighbors_of_face( Polytope &polytope, int f, std::set<int> &faces ) {
    std::set<int> edges;
    get_edges_of_face( polytope, f, edges );        //get all edges of the face
    std::for_each( std::begin(edges), std::end(edges), 	    //go through all edges and get their faces
					[&](auto &e) { 
                        get_faces_of_edge( polytope, e, faces );
                    } ); 
    faces.erase(f);         //remove the given face since it is not its own neighbor
}

