#pragma once

#define _USE_MATH_DEFINES
#include <cmath>

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


//a Point3D is a sphere with tiny radius
//can be used with GJK
struct Point3D : Sphere {
    Point3D( vec3 pos = vec3(0.0f, 0.0f, 0.0f) ) : Sphere(pos, SMALL_LENGTH) {}
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


//a Line3D is a capsule with tiny radius
//can be used with GJK
struct Line3D : Capsule {
    Line3D( vec3 start = vec3{0.0f, -0.5f, 0.0f}, vec3 end = vec3{0.0f, 0.5f, 0.0f}  ) 
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


//--------------------------------------------------------------------------------------

struct Polytope; 

struct PolytopePart : ICollider {
    PolytopePart() : ICollider() {};
};


struct VertexData {
    int              index;
    std::vector<int> edges;       //indices of all edges of this vertex
    std::vector<int> faces;       //indices of all faces of this vertex
    VertexData(int i, std::vector<int> e, std::vector<int> f) : index(i), edges(e), faces(f) {}
    VertexData & operator=(const VertexData & v) = default;
};


//Vertex of a polytope
//constexpr int MAX_NEIGHBORS = 8;   //adapt the max if you need more neighbors
struct Vertex : PolytopePart {
    Polytope*   polytope;
    const VertexData* data;

    Vertex(Polytope* p, const VertexData *d ) : PolytopePart(), polytope(p), data(d) {}
    Vertex & operator=(const Vertex & v) = default;
    vec3 support(vec3 dir);
};


struct EdgeData {
    int              index;
    std::vector<int> vertices;  //indices of the two vertices of this edge
    EdgeData(int i, std::vector<int> v) : index(i), vertices(v) {}
    EdgeData & operator=(const EdgeData & v) = default;
};


//Edge of a polytope
struct Edge : PolytopePart {
    Polytope* polytope;
    const EdgeData* data;

    Edge(Polytope* p, const EdgeData* d ) : PolytopePart(), polytope(p), data(d) {}
    Edge & operator=(const Edge & v) = default;

    bool contains_vertex( int v ) const {
        return data->vertices[0] == v || data->vertices[1] == v;
    }

    vec3 support(vec3 dir);
};


struct FaceData {
    int              index;
    std::vector<int> vertices;          //indices of all vertices of this face
    std::vector<int> edges;             //indices of all edges of this face
    std::vector<int> normal_vertices;   //indices of the points to use to compute the normal of this face
    std::vector<int> neighbors;         //indices of the faces that are neighbors of this face

    FaceData(int i, std::vector<int> v, std::vector<int> e, std::vector<int> n, std::vector<int> ne )
         : index(i), vertices(v), edges(e), normal_vertices(n), neighbors(ne) {}
    FaceData & operator=(const FaceData & v) = default;
};


//Face of a polytope
struct Face  : PolytopePart {
    Polytope*        polytope;
    const FaceData * data;

    Face(Polytope* p, const FaceData *d) : PolytopePart(), polytope(p), data(d) {}
    Face & operator=( const Face & v) = default;

    bool contains_edge( int e ) const {
        return std::find( std::begin(data->edges), std::end(data->edges), e) != std::end(data->edges);
    }

    bool contains_vertex(int v ) const {
        return std::find( std::begin(data->vertices), std::end(data->vertices), v) != std::end(data->vertices);
    }

    vec3 get_face_normal();
    void get_edge_vectors( std::vector<vec3>& vectors );
    vec3 support(vec3 dir);
};


struct Polygon;

//Polytope: Just a set of points plus adjencency information
struct Polytope : Collider {
    std::vector<vec3>     m_points;
    std::vector<Vertex>   m_vertices;
    std::vector<Edge>     m_edges;
    std::vector<Face>     m_faces;

    Polytope( vec3 pos = {0,0,0}, mat3 matRS = mat3(1.0f) ) : Collider(pos, matRS) {}

    //Dumb O(n) support function, just brute force check all points
    vec3 support(vec3 dir) {
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

    const std::vector<int>& get_vertex_edges( int v ) const;
    const std::vector<int>& get_face_edges( int f ) const;
    const std::vector<int>& get_face_neighbors( int f ) const;
    void get_vertex_neighbors(int v, std::set<int> &neighbors) const;
    void get_face_points( int f, std::vector<vec3> &points ) const;
    void get_edge_faces( int e, std::set<int> &faces) const;
    vec3 get_face_normal( int f ) const;
    void get_edge_vectors( std::vector<vec3> &edges) const;
};


struct Tetrahedron : Polytope {

    const static inline std::vector<VertexData> m_vertices_data =     //4 vertices, each is member of 3 edges and 3 faces	
                        {       VertexData{ 0, {0,2,3}, {0,1,3} }  //0
							, 	VertexData{ 1, {0,1,4}, {0,1,2} }  //1
							, 	VertexData{ 2, {1,2,5}, {0,2,3} }  //2
							, 	VertexData{ 3, {3,4,5}, {1,2,3} }  //3
                        };

    const static inline std::vector<EdgeData> m_edges_data =  //6 edges, each having 2 vertices
	                {   EdgeData{ 0, {0,1} }  //0
                    ,   EdgeData{ 1, {1,2} }  //1
                    ,   EdgeData{ 2, {0,2} }  //2
                    ,   EdgeData{ 3, {0,3} }  //3
                    ,   EdgeData{ 4, {1,3} }  //4
                    ,   EdgeData{ 5, {2,3} }  //5
                    }; 

	const static inline std::vector<FaceData> m_faces_data =  //4 faces, each has 3 vertices, 3 edges, 4 vertices for computing normals, 3 neighbor faces
                    {       FaceData{ 0, {0,1,2}, {0,1,2}, {1,0,2,0}, {1,2,3} }  //0
                        ,   FaceData{ 1, {0,3,1}, {0,3,4}, {3,0,1,0}, {0,2,3} }  //1
                        ,   FaceData{ 2, {1,3,2}, {1,4,5}, {3,1,2,1}, {0,1,3} }  //2
                        ,   FaceData{ 3, {2,3,0}, {2,5,3}, {3,2,0,2}, {0,1,2} }  //3
                    };

    Tetrahedron( vec3 p0, vec3 p1, vec3 p2, vec3 p3 )  : Polytope() {
	    m_points = { p0, p1, p2, p3 };
        for( const auto& data : m_vertices_data ) m_vertices.emplace_back(  this, &data );
        for( const auto& data : m_edges_data ) m_edges.emplace_back(  this, &data );
        for( const auto& data : m_faces_data ) m_faces.emplace_back( this, &data );
    };
};


//a triangle is a tertrahedron with tiny height
//can be used with GJK
struct Triangle3D : Tetrahedron {
    Triangle3D( vec3 p0, vec3 p1, vec3 p2 )  : Tetrahedron( p0, p1, p2, {0,0,0} ) {
        vec3 d0 = p2 - p0;
        vec3 d1 = p1 - p0;
        vec3 up = SMALL_LENGTH * normalize( cross( d0,  d1) );
        m_points[3] = (p0 + p1 + p2)*0.33333f + up;
    };
};


//a box is a polytope with 8 vertices
struct Box : Polytope {
    const static inline std::vector<VertexData> m_vertices_data = { 	//every vertex is member of 3 edges and 3 faces
                                VertexData{ 0, {0,2, 8}, {0,2,4} }   // 0  
							, 	VertexData{ 1, {0,1, 9}, {1,2,4} }   // 1
							, 	VertexData{ 2, {2,3,10}, {0,2,5} }   // 2
							, 	VertexData{ 3, {1,3,11}, {1,2,5} }   // 3
                            ,   VertexData{ 4, {4,6, 8}, {0,3,4} }   // 4
							, 	VertexData{ 5, {4,5, 9}, {1,3,4} }   // 5
							, 	VertexData{ 6, {6,7,10}, {0,3,5} }   // 6
							, 	VertexData{ 7, {5,7,11}, {1,3,5} }   // 7
						};

	const static inline std::vector<EdgeData> m_edges_data = {    //every edge has 2 vertices
                            EdgeData{  0, {0,1} } //0
                        ,   EdgeData{  1, {1,3} } //1
                        ,   EdgeData{  2, {0,2} } //2
                        ,   EdgeData{  3, {2,3} } //3
                        ,   EdgeData{  4, {4,5} } //4
                        ,   EdgeData{  5, {5,7} } //5
                        ,   EdgeData{  6, {4,6} } //6
                        ,   EdgeData{  7, {6,7} } //7
                        ,   EdgeData{  8, {0,4} } //8
                        ,   EdgeData{  9, {1,5} } //9
                        ,   EdgeData{ 10, {2,6} } //10
                        ,   EdgeData{ 11, {3,7} } //11
                        };           

    const static inline std::vector<FaceData> m_faces_data =  //6 faces, each having 4 vertices, 4 edges (clockwise), 4 vertices for computing normals, 4 neighbor faces
                    {   FaceData{ 0, {0,2,4,6}, {2,10,6,8},  {6,2,0,2}, {2,3,4,5} }   //0
                    ,   FaceData{ 1, {1,3,5,7}, {1,9,5,11},  {5,1,3,1}, {2,3,4,5} }   //1
                    ,   FaceData{ 2, {0,1,2,3}, {0,1,3,2},   {1,0,2,0}, {0,1,4,5} }   //2
                    ,   FaceData{ 3, {4,5,6,7}, {4,6,7,5},   {6,4,5,4}, {0,1,4,5} }   //3
                    ,   FaceData{ 4, {0,1,4,5}, {0,8,4,9},   {4,0,1,0}, {0,1,2,3} }   //4
                    ,   FaceData{ 5, {2,3,6,7}, {3,11,7,10}, {7,3,2,3}, {0,1,2,3} }   //5
                    };

    Box( vec3 pos = vec3(0.0f, 0.0f, 0.0f), mat3 matRS = mat3(1.0f) )  : Polytope( pos, matRS ) {
	    m_points = {    vec3(-0.5f, -0.5f, -0.5f), vec3(0.5f, -0.5f, -0.5f), vec3(-0.5f, -0.5f, 0.5f), vec3(0.5f, -0.5f, 0.5f),
                        vec3(-0.5f,  0.5f, -0.5f), vec3(0.5f,  0.5f, -0.5f), vec3(-0.5f,  0.5f, 0.5f), vec3(0.5f,  0.5f, 0.5f)};

        for( const auto& data : m_vertices_data ) m_vertices.emplace_back(  this, &data );
        for( const auto& data : m_edges_data ) m_edges.emplace_back( this, &data );
        for( const auto& data : m_faces_data ) m_faces.emplace_back( this, &data );
    };
};

//a quad is a box with tiny height
//can be used with GJK
struct Quad3D : Box {
    Quad3D( vec3 p0, vec3 p1, vec3 p2, vec3 p3 )  : Box() {   //points should lie in the same plane
        vec3 d0 = p2 - p0;
        vec3 d1 = p1 - p0;
        vec3 up = SMALL_LENGTH * normalize( cross( d0,  d1) );
	    m_points = { p0, p1, p2, p3, p0 + up, p1 + up, p2 + up, p3 + up };
    };
};

//a Polygon3D with N vertices, all vertices must lie in the same plane
//the polgon has a 3D body but is infinitely thin
//can be used with GJK
struct Polygon3D : Polytope {
    Polygon3D( std::vector<vec3>& points ) : Polytope() {
        vec3 d0 = points[2] - points[0];
        vec3 d1 = points[1] - points[0];
        vec3 up = SMALL_LENGTH * normalize( cross( d0,  d1) );

	    m_points = points;
        for( int i=0; i<points.size(); ++i ) {
            m_points.push_back(points[i] + up);
        }
    }
};


//-------------------------------------------------------------------------------------

//Polytope parts

vec3 Vertex::support(vec3 dir) {
    vec3 result = polytope->m_points[data->index];
    return polytope->matRS * result + polytope->pos; //convert support to world space
}

vec3 Edge::support(vec3 dir) {
    dir = polytope->matRS_inverse*dir; //find support in model space
    vec3 result = dot( dir, polytope->m_points[data->vertices[0]] ) > dot( dir, polytope->m_points[data->vertices[1]] ) 
                    ?  polytope->m_points[data->vertices[0]] : polytope->m_points[data->vertices[1]];

    return polytope->matRS * result + polytope->pos; //convert support to world space
}

vec3 Face::get_face_normal() {
    return polytope->get_face_normal( data->index );
}

void Face::get_edge_vectors( std::vector<vec3>& vectors ) {
    auto &pedges = polytope->m_edges;
    for( int e : data->edges ) {
        vec3 p0 = polytope->m_points[pedges[e].data->vertices[0]];
        vec3 p1 = polytope->m_points[pedges[e].data->vertices[1]];
        vectors.push_back( polytope->matRS * (p1 - p0) );
    }
}

vec3 Face::support(vec3 dir) {
    dir = polytope->matRS_inverse*dir; //find support in model space

    vec3 maxp = polytope->m_points[0];
    float max = dot( dir, maxp );
    for( auto i : data->vertices ) {
        float d = dot( dir, polytope->m_points[i] );
        if( d > max ) {
            maxp = polytope->m_points[i];
            max = d;
        }
    }

    return polytope->matRS * maxp + polytope->pos; //convert support to world space
}


//Polytope functions

//Return all neighboring vertices of a given vertex
void Polytope::get_vertex_neighbors(int v, std::set<int> &neighbors) const {
	const Vertex &vertex = m_vertices[v];	                    //edge info of vertex v
	std::for_each( std::begin(vertex.data->edges) , std::end(vertex.data->edges), //go through all edges that contain v
					[&]( auto &e ){ 
						auto &i = m_edges[e].data->vertices;       //access the vertices of the edge
						if(i[0]!=v) neighbors.insert(i[0]); //do not add the vertex itself, only true neighbor
						if(i[1]!=v) neighbors.insert(i[1]); 
					} );
}

//return all edges a given vertex is part of
const std::vector<int>& Polytope::get_vertex_edges( int v ) const {
	return m_vertices[v].data->edges;
}

//return all edges of a given face
const std::vector<int>& Polytope::get_face_edges( int f ) const {
	return m_faces[f].data->edges;
}

//return zero, one or two faces that contain a given edge
void Polytope::get_edge_faces( int e, std::set<int> &faces) const {
    for( int f=0; f<m_faces.size(); ++f) {
        const Face &face = m_faces[f];
        if( face.contains_edge(e) )     //it contains the edge
            faces.insert(f);
    }
}

//return all neighboring faces of a given face
const std::vector<int> & Polytope::get_face_neighbors( int f ) const {
    return m_faces[f].data->neighbors;        //get all edges of the face
}

//get the normal of a given face
vec3 Polytope::get_face_normal( int f ) const {
    auto &n = m_faces[f].data->normal_vertices;
    return cross( matRS * (m_points[n[0]] - m_points[n[1]]), matRS * (m_points[n[2]] - m_points[n[3]] ) );
}

//return a list of vectors going along the edges of the polytope
void Polytope::get_edge_vectors( std::vector<vec3> &edges) const {
    std::for_each( std::begin(m_edges), std::begin(m_edges),
                    [&,this]( auto &e) {
                        vec3 v0 = this->m_points[e.data->vertices[0]];
                        vec3 v1 = this->m_points[e.data->vertices[1]];
                        edges.push_back( matRS * ( v1 - v0 ) );
                    });
}

//return a list with the coordinates of the vertices of a given face
void Polytope::get_face_points( int f, std::vector<vec3> &points ) const {
    const Face &face = m_faces[f];
    for( auto i : face.data->vertices) {
        points.push_back( matRS * m_points[i] + pos );
    }
}


