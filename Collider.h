#pragma once

//Kevin's simple collider objects for collision detection
//Different shapes which inherit from Collider and implement
//support() function for use in GJK

using namespace glm;

constexpr float SMALL_LENGTH = 1.0e-6f;

struct ICollider {
    ICollider(){};
    virtual vec3 support(vec3 dir) = 0;
};


//Base struct for all collision shapes
struct Collider : ICollider {
    vec3    pos;            //origin in world space
    mat3    matRS;          //rotation/scale component of model matrix
    mat3    matRS_inverse; 

    Collider( vec3 p = {0,0,0}, mat3 m = mat3(1.f) ) {
        pos = p;
        matRS = m;
        matRS_inverse = inverse( m );
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

    Sphere( vec3 pos = vec3(0.0f, 0.0f, 0.0f), float radius = 1.0f) : Collider(pos, mat3(1.0f)), r(radius) {};

    vec3 support(vec3 dir){
        return normalize(dir)*r + pos;
    }
};


//a point is a sphere with tiny radius
struct Point : Sphere {
    Point( vec3 pos = vec3(0.0f, 0.0f, 0.0f) ) : Sphere(pos, SMALL_LENGTH) {}
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

    Capsule(    vec3 pos = vec3{0.0f, 0.0f, 0.0f}, mat3 matRS = mat3(1.0f)
            ,   float radius = 0.2f, float yb = -0.5f, float yc = 0.5 ) 
                : Collider( pos, matRS ), r(radius), y_base(yb), y_cap(yc)  {}

    vec3 support(vec3 dir){
        dir = matRS_inverse*dir; //find support in model space

        vec3 result = normalize(dir)*r;
        result.y += (dir.y>0) ? y_cap : y_base;

        return matRS*result + pos; //convert support to world space
    }
};


//a line is a capsule with tiny radius
struct Line : Capsule {
    Line( vec3 start = vec3{0.0f, -0.5f, 0.0f}, vec3 end = vec3{0.0f, 0.5f, 0.0f}  ) 
            : Capsule( (start + end)*0.5f, mat3(1.0), 1.0e-6f ) {
        vec3 vector = end - start;
        vec3 dir = normalize( vector );
        if( length(dir) < 0.9f ) return;

        vec3 up = {0.0f, 1.0f, 0.0f};
        vec3 naxis = cross( up, dir );
        if( length( naxis ) < 1.0e-9 )  {
            up = {1.0f, 0.0f, 0.0f};
            naxis = cross( up, dir );
        }
        vec3 xaxis = cross( dir, naxis);
        vec3 zaxis = cross( xaxis, dir);

        matRS = mat3( xaxis, dir, zaxis );
        matRS = matRS * mat3( scale( mat4(1.0f), vec3(1.0f, length( vector ), 1.0f) ) );
        matRS_inverse = inverse( matRS );
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


struct Polytope; 

struct PolytopePart : ICollider {
    Polytope        *polytope;
};


//Vertex of a polytope
//constexpr int MAX_NEIGHBORS = 8;   //adapt the max if you need more neighbors
struct Vertex : ICollider {
    Polytope* polytope;
    std::vector<int> edges;       //indices of all edges of this vertex
    std::vector<int> faces;       //indices of all faces of this vertex

    Vertex(Polytope* p, std::vector<int> e, std::vector<int> f) : ICollider(), polytope(p), edges(e), faces(f) {}

    Vertex & operator=(const Vertex & v) = default;

    vec3 support(vec3 dir) {
        return {0,0,0};
    }
};


//Edge of a polytope
struct Edge : ICollider {
    Polytope         *polytope;
    std::vector<int>  vertices;  //indices of the two vertices of this edge

    Edge(Polytope* p, std::vector<int> v) : ICollider(), polytope(p), vertices(v) {}

    Edge & operator=(const Edge & v) = default;

    bool contains_vertex( int v ) {
        return vertices[0] == v || vertices[1] == v;
    }

    vec3 support(vec3 dir) {
        return {0,0,0};
    }
};


//Face of a polytope
struct Face  : ICollider {
    Polytope          *polytope;
    std::vector<int>  vertices;      //indices of all vertices of this face
    std::vector<int>  edges;         //indices of all edges of this face
    std::vector<int>  normal;        //indices of the points to use to compute the cormal of this face

    Face(Polytope* p, std::vector<int> v, std::vector<int> e, std::vector<int> n) 
        : ICollider(), polytope(p), vertices(v), edges(e), normal(n) {}

    Face & operator=(const Face & v) = default;

    bool contains_edge( int v ) {
        return std::find( edges.begin(), edges.end(), v) != edges.end();
    }

    bool contains_vertex(int v ) {
        return std::find( std::begin(vertices), std::end(vertices), v) != std::end(vertices);
    }

    vec3 support(vec3 dir) {
        return {0,0,0};
    }
};


struct Polygon;

//Polytope: Just a set of points plus adjencency information
struct Polytope : Collider {
    std::vector<vec3>   m_points;
    std::vector<Vertex> m_vertices;
    std::vector<Edge>   m_edges;
    std::vector<Face>   m_faces;

    Polytope( vec3 pos = {0,0,0}, mat3 matRS = mat3(1.0f) ) : Collider(pos, matRS) {}

    //Dumb O(n) support function, just brute force check all points
    vec3 support(vec3 dir){
        dir = matRS_inverse*dir;

        vec3 furthest_point = m_points[0]; 
        float max_dot = dot(furthest_point, dir);

        if( true ) {           //if( neighbors == nullptr ) {  //replace the if statement so this branch is only taken if no neighbor info available
            std::for_each(  std::begin(m_points), std::end(m_points), 
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

    void get_vertex_neighbors(int v, std::set<int> &neighbors);
    std::vector<int>& get_vertex_edges( int v );
    std::vector<int>& get_face_edges( int f );
    void get_face_points( int f, std::vector<vec3> &points );
    void get_edge_faces( int e, std::set<int> &faces);
    void get_face_neighbors( int f, std::set<int> &faces );
    vec3 get_face_normal( int f );
    void get_edge_vectors( std::vector<vec3> &edges);
    Polygon && face_to_polygon(int f);
};


struct Tetrahedron : Polytope {
    Tetrahedron( vec3 p0, vec3 p1, vec3 p2, vec3 p3 )  : Polytope() {
	    m_points = { p0, p1, p2, p3 };

	    m_vertices = 	{ 	    Vertex{ this, {0,2,3}, {0,1,3} } //every vertex is member of 3 edges and 3 faces
							, 	Vertex{ this, {0,1,4}, {0,1,2} }
							, 	Vertex{ this, {1,2,5}, {0,2,3} } 
							, 	Vertex{ this, {3,4,5}, {1,2,3} } 
                        };

	    m_edges = {     Edge{ this, {0,1} }	    //6 edges, each having 2 vertices
                    ,   Edge{ this, {1,2} }
                    ,   Edge{ this, {2,0} }
                    ,   Edge{ this, {0,3} }
                    ,   Edge{ this, {1,3} }
                    ,   Edge{ this, {2,3} }
                  }; 

	    m_faces =   {       Face{ this, {0,1,2}, {0,1,2}, {1,0,2,0} }  //4 faces, each has 3 vertices, 3 edges, 4 vertices for normals
                        ,   Face{ this, {0,3,1}, {0,3,4}, {3,0,1,0} }
                        ,   Face{ this, {1,3,2}, {1,4,5}, {3,1,2,1} }
                        ,   Face{ this, {2,3,0}, {2,3,5}, {3,2,0,2} }
                    };
    };
};


//a triangle is a tertrahedron with tiny height
struct Triangle : Tetrahedron {
    Triangle( vec3 p0, vec3 p1, vec3 p2 )  : Tetrahedron( p0, p1, p2, {0,0,0} ) {
        vec3 d0 = p2 - p0;
        vec3 d1 = p1 - p0;
        vec3 up = SMALL_LENGTH * normalize( cross( d0,  d1) );
        m_points[3] = (p0 + p1 + p2)*0.33333f + up;
    };
};


//a box is a polytope with 8 vertices
struct Box : Polytope {
    Box( vec3 pos = vec3(0.0f, 0.0f, 0.0f), mat3 matRS = mat3(1.0f) )  : Polytope( pos, matRS ) {
	    m_points = {    vec3(-0.5f, -0.5f, -0.5f), vec3(0.5f, -0.5f, -0.5f), vec3(-0.5f, -0.5f, 0.5f), vec3(0.5f, -0.5f, 0.5f),
                        vec3(-0.5f,  0.5f, -0.5f), vec3(0.5f,  0.5f, -0.5f), vec3(-0.5f,  0.5f, 0.5f), vec3(0.5f,  0.5f, 0.5f)};
	    m_vertices = 	{ 	    //every vertex is member of 3 edges and 3 faces
                                Vertex{ this, {0,2, 8}, {0,2,4} }   // 0  
							, 	Vertex{ this, {0,1, 9}, {1,2,4} }   // 1
							, 	Vertex{ this, {2,3,10}, {0,2,5} }   // 2
							, 	Vertex{ this, {1,3,11}, {1,2,5} }   // 3
                            ,   Vertex{ this, {4,6, 8}, {0,3,4} }   // 4
							, 	Vertex{ this, {4,5, 9}, {1,3,4} }   // 5
							, 	Vertex{ this, {6,7,10}, {0,3,5} }   // 6
							, 	Vertex{ this, {5,7,11}, {1,3,5} }   // 7
						};
	    m_edges = {     Edge{ this, {0,1} } //0
                    ,   Edge{ this, {1,2} } //1
                    ,   Edge{ this, {0,2} } //2
                    ,   Edge{ this, {2,3} } //3
                    ,   Edge{ this, {4,5} } //4
                    ,   Edge{ this, {5,7} } //5
                    ,   Edge{ this, {4,6} } //6
                    ,   Edge{ this, {6,7} } //7
                    ,   Edge{ this, {0,4} } //8
                    ,   Edge{ this, {1,5} } //9
                    ,   Edge{ this, {2,6} } //10
                    ,   Edge{ this, {3,7} } //11
                  }; 

                        //6 faces, each having 4 vertices, 4 edges, 4 vertices for normals
	    m_faces =  {    Face{ this, {0,2,4,6}, {2,4,6,8},   {6,2,0,2} }   //0
                    ,   Face{ this, {1,3,5,7}, {1,5,9,11},  {5,1,3,1} }   //1
                    ,   Face{ this, {0,1,2,3}, {0,1,2,3},   {1,0,2,0} }   //2
                    ,   Face{ this, {4,5,6,7}, {4,5,6,7},   {6,4,5,4} }   //3
                    ,   Face{ this, {0,1,4,5}, {0,4,8,9},   {4,0,1,0} }   //4
                    ,   Face{ this, {2,3,6,7}, {3,7,10,11}, {7,3,2,3} }   //5
                    };
    };
};

//a quad is a box with tiny height
struct Quad : Box {
    Quad( vec3 p0, vec3 p1, vec3 p2, vec3 p3 )  : Box() {   //points should lie in the same plane
        vec3 d0 = p2 - p0;
        vec3 d1 = p1 - p0;
        vec3 up = SMALL_LENGTH * normalize( cross( d0,  d1) );
	    m_points = { p0, p1, p2, p3, p0 + up, p1 + up, p2 + up, p3 + up };
    };
};


struct Polygon : Polytope {

    Polygon( std::vector<vec3>& points ) : Polytope() {
        vec3 d0 = points[2] - points[0];
        vec3 d1 = points[1] - points[0];
        vec3 up = SMALL_LENGTH * normalize( cross( d0,  d1) );

	    m_points = points;
        for( int i=0; i<points.size(); ++i ) {
            m_points.push_back(points[i] + up);
        }
    }
};


//implementations


//Polytope functions

//Return all neighboring vertices of a given vertex
void Polytope::get_vertex_neighbors(int v, std::set<int> &neighbors) {
	Vertex &vertex = m_vertices[v];	                    //edge info of vertex v
	std::for_each( std::begin(vertex.edges) , std::end(vertex.edges), //go through all edges that contain v
					[&]( auto &e ){ 
						auto &i = m_edges[e].vertices;       //access the vertices of the edge
						if(i[0]!=v) neighbors.insert(i[0]); //do not add the vertex itself, only true neighbor
						if(i[1]!=v) neighbors.insert(i[1]); 
					} );
}

//return all edges a given vertex is part of
std::vector<int>& Polytope::get_vertex_edges( int v ) {
	Vertex &vertex = m_vertices[v];  		        //membership info of this vertex
	return vertex.edges;
}

//return all edges of a given face
std::vector<int>& Polytope::get_face_edges( int f ) {
	Face &face = m_faces[f];                         //information of face f
	return face.edges;
}

//return zero, one or two faces that contain a given edge
void Polytope::get_edge_faces( int e, std::set<int> &faces) {
    for( int f=0; f<m_faces.size(); ++f) {
        Face &face = m_faces[f];
        if( face.contains_edge(e) )     //it contains the edge
            faces.insert(f);
    }
}

//return all neighboring faces of a given face
void Polytope::get_face_neighbors( int f, std::set<int> &faces ) {
    auto& edges = m_faces[f].edges;        //get all edges of the face
    std::for_each( std::begin(edges), std::end(edges), 	    //go through all edges and get their faces
					[&](auto &e) { 
                        get_edge_faces( e, faces );
                    } ); 
    faces.erase(f);         //remove the given face since it is not its own neighbor
}

vec3 Polytope::get_face_normal( int f ) {
    auto &n = m_faces[f].normal;
    return cross( m_points[n[0]] - m_points[n[1]], m_points[n[2]] - m_points[n[3]] );
}

void Polytope::get_edge_vectors( std::vector<vec3> &edges) {
    std::for_each( std::begin(m_edges), std::begin(m_edges),
                    [&,this]( auto &e) {
                        vec3 v0 = this->m_points[e.vertices[0]];
                        vec3 v1 = this->m_points[e.vertices[1]];
                        edges.push_back( v1 - v0 );
                    });
}

void Polytope::get_face_points( int f, std::vector<vec3> &points ) {
    Face &face = m_faces[f];
    for( auto i : face.vertices) {
        points.push_back( m_points[i]);
    }
}

Polygon && Polytope::face_to_polygon(int f) {
    std::vector<vec3> points;
    get_face_points(f, points);
    return std::move( Polygon{ points } );
}

