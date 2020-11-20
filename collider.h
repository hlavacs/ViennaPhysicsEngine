#pragma once

#define _USE_MATH_DEFINES
#include <cmath>

using namespace glm;

#include "define.h"
#include "pluecker.h"



struct ICollider {
    ICollider(){};
    virtual vec3 support(vec3 dir) = 0;
};


//Base struct for all collision shapes
struct Collider : ICollider {
    vec3    m_pos;            //origin in world space
    mat3    m_matRS;          //rotation/scale component of model matrix
    mat3    m_matRS_inverse; 

    Collider( vec3 p = {0,0,0}, mat3 m = mat3(1.f) ) : ICollider() {
        m_pos = p;
        m_matRS = m;
        m_matRS_inverse = inverse( m );
    };
};

//BBox: AABB + Orientation matrix
struct BBox : Collider {
    vec3 m_min, m_max; //Assume these are axis aligned!

    vec3 support(vec3 dir){
        dir = m_matRS_inverse*dir; //find support in model space

        vec3 result;
        result.x = (dir.x>0) ? m_max.x : m_min.x;
        result.y = (dir.y>0) ? m_max.y : m_min.y;
        result.z = (dir.z>0) ? m_max.z : m_min.z;

        return m_matRS*result + m_pos; //convert support to world space
    }
};

//Sphere: NB Does not use RS matrix, scale the radius directly!
struct Sphere : Collider {
    float m_r;

    Sphere( vec3 pos = vec3(0.0f, 0.0f, 0.0f), float radius = 1.0f) : Collider(pos, mat3(1.0f)), m_r(radius) {};

    vec3 support(vec3 dir){
        return normalize(dir)*m_r + m_pos;
    }
};


//a Point3D is a sphere with tiny radius
//can be used with GJK
struct Point3D : Sphere {
    Point3D( vec3 pos = vec3(0.0f, 0.0f, 0.0f) ) : Sphere(pos, EPS) {}
    pluecker_point pluecker() { return { m_pos, 1}; };
};


//a 1d line segment, do not use ith GJK
struct Point : Collider {
    Point( vec3 p ) : Collider(p) {}
    Point & operator=(const Point & l) = default;
    pluecker_point plueckerW() { return { m_pos, 1}; };

    vec3 support(vec3 dir) {
        return m_pos; 
    }
};



//Cylinder: Height-aligned with y-axis (rotate using matRS)
struct Cylinder : Collider {
    float m_r, m_y_base, m_y_cap;

    vec3 support(vec3 dir){
        dir = m_matRS_inverse*dir; //find support in model space
        vec3 dir_xz = vec3(dir.x, 0, dir.z);
        vec3 result = normalize(dir_xz)*m_r;
        result.y = (dir.y>0) ? m_y_cap : m_y_base;
        return m_matRS*result + m_pos; //convert support to world space
    }
};


//Capsule: Height-aligned with y-axis
struct Capsule : Collider {
    float m_r, m_y_base, m_y_cap;

    Capsule(    vec3 pos = vec3{0.0f, 0.0f, 0.0f}, mat3 matRS = mat3(1.0f)
            ,   float radius = 0.2f, float yb = -0.5f, float yc = 0.5 ) 
                    : Collider( pos, matRS ), m_r(radius), m_y_base(yb), m_y_cap(yc)  {}

    vec3 support(vec3 dir){
        dir = m_matRS_inverse*dir; //find support in model space
        vec3 result = normalize(dir)*m_r;
        result.y += (dir.y>0) ? m_y_cap : m_y_base;
        return m_matRS*result + m_pos; //convert support to world space
    }
};


//a Line3D is a capsule with tiny radius
//can be used with GJK
struct Line3D : Capsule {
    Line3D( vec3 start = vec3{0.0f, -0.5f, 0.0f}, vec3 end = vec3{0.0f, 0.5f, 0.0f}  ) 
            : Capsule( (start + end)*0.5f, mat3(1.0), EPS ) {
        vec3 vector = end - start;
        vec3 dir = normalize( vector );
        if( length(dir) < 0.9f ) return;

        vec3 up = {0.0f, 1.0f, 0.0f};
        vec3 naxis = cross( up, dir );
        if( length( naxis ) < EPS )  {
            up = {1.0f, 0.0f, 0.0f};
            naxis = cross( up, dir );
        }
        vec3 xaxis = cross( dir, naxis);
        vec3 zaxis = cross( xaxis, dir);

        m_matRS = mat3( xaxis, dir, zaxis );
        m_matRS = m_matRS * mat3( scale( mat4(1.0f), vec3(1.0f, length( vector ), 1.0f) ) );
        m_matRS_inverse = inverse( m_matRS );
    }

    pluecker_line plueckerW() { 
        vec3 dir = m_matRS*vec3(0,1,0);  //capsule in local space along y axis
        return { dir, cross( m_pos, m_pos + dir) }; 
    };

};

//a 1d line segment, do not use ith GJK
struct Line : Collider {
    vec3 m_dir;  //

    Line( vec3 p0, vec3 p1 ) : Collider(p0), m_dir(p1 - p0) {}
    Line & operator=(const Line & l) = default;

    pluecker_line plueckerW() {
        vec3 dir = m_matRS*m_dir;
        return { dir, cross( m_pos, m_pos + dir) }; 
    };

    vec3 support(vec3 dir) {
        dir = m_matRS_inverse*dir; //find support in model space
        vec3 result = dot( dir, m_dir ) < 0.0f ?  vec3{0,0,0} : m_dir;
        return m_matRS*result + m_pos; //convert support to world space
    }
};



//--------------------------------------------------------------------------------------

struct Polytope; 

struct PolytopePart : ICollider {
    PolytopePart() : ICollider() {};
};


struct VertexData {
    int              m_index;       //index of this vertex
    std::vector<int> m_faces;       //indices of all faces of this vertex
    std::vector<int> m_neighbors;   //indices of all vertex neighbors of this vertex
    VertexData(int i, std::vector<int> f, std::vector<int> ne) 
        : m_index(i), m_faces(f), m_neighbors(ne) {}
    VertexData & operator=(const VertexData & v) = default;
};


//Vertex of a polytope
//constexpr int MAX_NEIGHBORS = 8;   //adapt the max if you need more neighbors
struct Vertex : PolytopePart {
    Polytope*           m_polytope;
    const VertexData*   m_data;
    vec3                m_pointL;

    Vertex(Polytope* p, const VertexData *d, vec3 pointL ) : PolytopePart(), m_polytope(p), m_data(d), m_pointL(pointL) {}
    Vertex & operator=(const Vertex & v) = default;
    vec3 pointL() { return m_pointL; };
    vec3 pointW();
    const std::vector<int> & get_vertex_neighbors() const { assert(m_data!=nullptr); return m_data->m_neighbors;};
    pluecker_point plueckerW();
    vec3 support(vec3 dir);
};


struct FaceData {
    int              m_index;
    std::vector<int> m_vertices;          //indices of all vertices of this face
    std::vector<int> m_normal_vertices;   //indices of the points to use to compute the normal of this face
    std::vector<int> m_neighbors;         //indices of the faces that are neighbors of this face

    FaceData(int i, std::vector<int> v, std::vector<int> n, std::vector<int> ne )
         : m_index(i), m_vertices(v), m_normal_vertices(n), m_neighbors(ne) {}
    FaceData & operator=(const FaceData & v) = default;
};


//Face of a polytope
struct Face  : PolytopePart {
    Polytope*        m_polytope;
    const FaceData * m_data;

    Face(Polytope* p, const FaceData *d) : PolytopePart(), m_polytope(p), m_data(d) {}

    bool contains_vertex(int v ) const {
        auto &dm = m_data->m_vertices;
        return std::find( std::begin(dm), std::end(dm), v) != std::end(dm);
    }

    vec3 normalW() const;
    void pointsW( std::vector<vec3> &points ) const;
    void edge_vectorsW( std::vector<vec3> & vectors ) const ;
    void get_edgesW( std::vector<Line> & edges );
    const std::vector<int> & neighbors() const { return m_data->m_neighbors; }
    pluecker_plane plueckerW();
    vec3 support(vec3 dir);
};


//Polytope: Just a set of points plus adjencency information
struct Polytope : Collider {
    std::vector<Vertex>   m_vertices;
    std::vector<Face>     m_faces;

    Polytope( vec3 pos = {0,0,0}, mat3 matRS = mat3(1.0f) ) : Collider(pos, matRS) {}

    Polytope( std::vector<vec3>& points, vec3 pos = {0,0,0}, mat3 matRS = mat3(1.0f) ) : Collider(pos, matRS) {
        for( const auto& p : points ) m_vertices.emplace_back(  this, nullptr, p );
    }

    Polytope(     const std::vector<vec3>& points
                , const std::vector<VertexData>& vertex_data
                , const std::vector<FaceData>& face_data 
                , vec3 pos = {0,0,0}, mat3 matRS = mat3(1.0f) ) : Collider(pos, matRS) {
        int i=0;
        for( auto &v : vertex_data ) m_vertices.emplace_back( this, &v, points[i++] );
        for( auto &f : face_data )   m_faces.emplace_back( this, &f );
    }

    vec3 support(vec3 dir) {
        if( m_vertices.empty()) return {0,0,0};

        dir = m_matRS_inverse*dir;
        vec3 furthest_point = m_vertices[0].pointL(); 
        float max_dot = dot(furthest_point, dir);

        if( true ) {
            std::for_each(  std::begin(m_vertices), std::end(m_vertices), 
                            [&]( auto &vertex) {
                                float d = dot(vertex.pointL(), dir);
                                if(d>max_dot){
                                    max_dot = d;
                                    furthest_point = vertex.pointL();
                                }
                            } );
        } else {
            //ADD YOUR CODE HERE TO ITERATE THROUGH NEIGHBORS rather than iterate through all points
        }

        vec3 result = m_matRS*furthest_point + m_pos; //convert support to world space
        return result;
    }

    Vertex & vertex( int v) { return m_vertices[v]; }

    Face & face( int f ) { return m_faces[f]; }

    void edge_vectorsW( std::vector<vec3> & vectors) const {
        for( auto & face : m_faces ) {
            face.edge_vectorsW( vectors );
        }
    }

    void edgesW( std::vector<Line> & edges ) {
        for( auto & face : m_faces ) {
            face.get_edgesW( edges );
        }
    }

};


struct Tetrahedron : Polytope {

    const static inline std::vector<VertexData> m_vertices_data =  //4 vertices, each is member of 3 faces and has 3 neighbors
                        {       VertexData{ 0, {0,1,3}, {1,2,3} }  //0
							, 	VertexData{ 1, {0,1,2}, {0,2,3} }  //1
							, 	VertexData{ 2, {0,2,3}, {0,1,3} }  //2
							, 	VertexData{ 3, {1,2,3}, {0,1,2} }  //3
                        };

	const static inline std::vector<FaceData> m_faces_data =  //4 faces, each has 3 vertices (clockwise), 4 vertices for computing normals, 3 neighbor faces
                    {       FaceData{ 0, {0,1,2}, {1,0,2,0}, {1,2,3} }  //0
                        ,   FaceData{ 1, {0,3,1}, {3,0,1,0}, {0,2,3} }  //1
                        ,   FaceData{ 2, {1,3,2}, {3,1,2,1}, {0,1,3} }  //2
                        ,   FaceData{ 3, {2,3,0}, {3,2,0,2}, {0,1,2} }  //3
                    };

    const static inline std::vector<vec3> m_points_data = {{-1,0,0},{1,0,0},{0,0,1},{0,1,0.5}};

    Tetrahedron()  : Polytope( m_points_data, m_vertices_data, m_faces_data ) {}

    Tetrahedron( vec3 p0, vec3 p1, vec3 p2, vec3 p3 )  : Polytope( m_points_data, m_vertices_data, m_faces_data ) {
        m_vertices[0].m_pointL = p0;
        m_vertices[1].m_pointL = p1;
        m_vertices[2].m_pointL = p2;
        m_vertices[3].m_pointL = p3;
    };
};


//a triangle is a tertrahedron with tiny height
//can be used with GJK
struct Triangle3D : Tetrahedron {
    Triangle3D( vec3 p0, vec3 p1, vec3 p2 )  : Tetrahedron( p0, p1, p2, {0,0,0} ) {
        vec3 d0 = p2 - p0;
        vec3 d1 = p1 - p0;
        vec3 up = EPS * normalize( cross( d0,  d1) );
        m_vertices[3].m_pointL = (p0 + p1 + p2)*0.33333f + up;
    };
};


//a box is a polytope with 8 vertices
struct Box : Polytope {
                                //every vertex is member of 3 faces and has 3 neighbors
    const static inline std::vector<VertexData> m_vertices_data = 
                        { 
                                VertexData{ 0, {0,2,4}, {1,2,4} }   // 0  
							, 	VertexData{ 1, {1,2,4}, {0,3,5} }   // 1
							, 	VertexData{ 2, {0,2,5}, {0,3,6} }   // 2
							, 	VertexData{ 3, {1,2,5}, {1,2,7} }   // 3
                            ,   VertexData{ 4, {0,3,4}, {0,5,6} }   // 4
							, 	VertexData{ 5, {1,3,4}, {1,4,7} }   // 5
							, 	VertexData{ 6, {0,3,5}, {2,4,7} }   // 6
							, 	VertexData{ 7, {1,3,5}, {3,5,6} }   // 7
						};

                        //6 faces, each having 4 vertices (clockwise), 4 vertices for computing normals, 4 neighbor faces
    const static inline std::vector<FaceData> m_faces_data =  
                    {   FaceData{ 0, {0,2,4,6}, {6,2,0,2}, {2,3,4,5} }   //0
                    ,   FaceData{ 1, {1,3,5,7}, {5,1,3,1}, {2,3,4,5} }   //1
                    ,   FaceData{ 2, {0,1,2,3}, {1,0,2,0}, {0,1,4,5} }   //2
                    ,   FaceData{ 3, {4,5,6,7}, {6,4,5,4}, {0,1,4,5} }   //3
                    ,   FaceData{ 4, {0,1,4,5}, {4,0,1,0}, {0,1,2,3} }   //4
                    ,   FaceData{ 5, {2,3,6,7}, {7,3,2,3}, {0,1,2,3} }   //5
                    };

    const static inline std::vector<vec3> m_points_data = //3D coordinates in local space
                    {   vec3(-0.5f, -0.5f, -0.5f), vec3(0.5f, -0.5f, -0.5f)
                    ,   vec3(-0.5f, -0.5f, 0.5f), vec3(0.5f, -0.5f, 0.5f)
                    ,   vec3(-0.5f,  0.5f, -0.5f), vec3(0.5f,  0.5f, -0.5f)
                    ,   vec3(-0.5f,  0.5f, 0.5f), vec3(0.5f,  0.5f, 0.5f)
                    };

    Box( vec3 pos = vec3(0.0f, 0.0f, 0.0f), mat3 matRS = mat3(1.0f) )  
        : Polytope( m_points_data, m_vertices_data, m_faces_data, pos, matRS ) {};
};


//a quad is a box with tiny height
//can be used with GJK
struct Quad3D : Box {
    Quad3D( vec3 p0, vec3 p1, vec3 p2, vec3 p3 )  : Box() {   //points should lie in the same plane
        vec3 d0 = p2 - p0;
        vec3 d1 = p1 - p0;
        vec3 up = EPS * normalize( cross( d0,  d1) );
        std::vector<vec3> points = {p0, p1, p2, p3, p0 + up, p1 + up, p2 + up, p3 + up};
        int i = 0;
        for( auto& vertex : m_vertices ) vertex.m_pointL = points[i++];
    };
};


//a Polygon3D with N vertices, all vertices must lie in the same plane
//the polgon has a 3D body but is infinitely thin
//can be used with GJK
struct Polygon3D : Polytope {
    Polygon3D( std::vector<vec3>& points ) : Polytope() {
        vec3 d0 = points[2] - points[0];
        vec3 d1 = points[1] - points[0];
        vec3 up = EPS * normalize( cross( d0,  d1) );

        for( auto &p : points ) m_vertices.emplace_back( this, nullptr, p );
        for( auto &p : points ) m_vertices.emplace_back( this, nullptr, p + up );
    }
};


//-------------------------------------------------------------------------------------

//Polytope parts

vec3 Vertex::pointW() {
    return m_polytope->m_matRS * pointL() + m_polytope->m_pos;
}

pluecker_point Vertex::plueckerW() { 
    return { pointW(), 1.0f}; 
};

vec3 Vertex::support(vec3 dir) {
    return pointW(); //convert support to world space
}

//get the normal of the face
vec3 Face::normalW() const {
    auto &n = m_data->m_normal_vertices;
    Vertex v0 = m_polytope->m_vertices[n[0]];
    Vertex v1 = m_polytope->m_vertices[n[1]];
    Vertex v2 = m_polytope->m_vertices[n[2]];
    Vertex v3 = m_polytope->m_vertices[n[3]];
    return cross( v0.pointW() - v1.pointW(), v2.pointW() - v3.pointW());
}

//return a list with the coordinates of the vertices of a given face in world coordinates
void Face::pointsW( std::vector<vec3> &points ) const {
    for( auto i : m_data->m_vertices) {
        points.push_back( m_polytope->m_vertices[i].pointW() );
    }
}

//return a list of vectors going along the edges of the face
void Face::edge_vectorsW( std::vector<vec3> &vectors ) const {
    int v0 = m_data->m_vertices.back();
    for( int v : m_data->m_vertices ) {
        vectors.push_back( m_polytope->m_vertices[v].pointW() - m_polytope->m_vertices[v0].pointW() );
        v0 = v;
    }
}

//return a list of edges of this face
void Face::get_edgesW( std::vector<Line> & edges ) {
    int v0 = m_data->m_vertices.back();
    for( int v : m_data->m_vertices ) {
        edges.emplace_back( m_polytope->m_vertices[v0].pointW(), m_polytope->m_vertices[v].pointW() );
        v0 = v;
    }
}

pluecker_plane Face::plueckerW() {
    vec3 q = m_polytope->m_vertices[m_data->m_vertices[0]].pointW();
    vec3 normal = normalW();
    return { normal, -1.0f * dot( normal, q)};
}

vec3 Face::support(vec3 dir) {
    dir = m_polytope->m_matRS_inverse*dir; //find support in model space
    vec3 maxL = m_polytope->m_vertices[0].pointL();
    float max = dot( dir, maxL );
    for( auto i : m_data->m_vertices ) {
        float d = dot( dir, m_polytope->m_vertices[i].pointL() );
        if( d > max ) {
            maxL = m_polytope->m_vertices[i].pointL();
            max = d;
        }
    }
    return m_polytope->m_matRS * maxL + m_polytope->m_pos; //convert support to world space
}


//Polytope functions




//-------------------------------------------------------------------------------------


