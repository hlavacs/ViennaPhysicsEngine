#pragma once

#define _USE_MATH_DEFINES
#include <cmath>

#include "glm/glm/glm.hpp"
#include "glm/glm/ext.hpp"

using namespace glm;

#include <vector>

#include "define.h"
#include "pluecker.h"


struct ICollider {
    ICollider(){};
    virtual vec3 support(vec3 dir) = 0;
};


//Base struct for all collision shapes
struct Collider : ICollider {
    protected:
    vec3    m_pos;            //origin in world space
    mat3    m_matRS;          //rotation/scale component of model matrix
    mat3    m_matRS_inverse; 

    public:
    Collider( vec3 p = {0,0,0}, mat3 m = mat3(1.f) ) : ICollider() {
        m_pos = p;
        m_matRS = m;
        m_matRS_inverse = inverse( m );
    };
    vec3 &pos(){ return m_pos; }
    mat3 &matRS() { return m_matRS; };
    mat3 &matRS_inverse() { return m_matRS_inverse; };    
    vec3 posL2W( vec3 posL ){ return m_matRS * posL + m_pos; }
    vec3 posW2L( vec3 posW ){ return m_matRS_inverse * ( posW - m_pos ); }
    vec3 dirW2L( vec3 dirW ){ return m_matRS_inverse * dirW; }
};

//BBox: AABB + Orientation matrix
struct BBox : Collider {
    vec3 m_min, m_max; //Assume these are axis aligned!

    vec3 support(vec3 dirW){
        auto dirL = dirW2L(dirW); //find support in model space

        vec3 result;
        result.x = (dirL.x>0) ? m_max.x : m_min.x;
        result.y = (dirL.y>0) ? m_max.y : m_min.y;
        result.z = (dirL.z>0) ? m_max.z : m_min.z;

        return posL2W(result); //convert support to world space
    }
};

//Sphere: NB Does not use RS matrix, scale the radius directly!
struct Sphere : Collider {
    float m_r;

    Sphere( vec3 pos = vec3(0.0f, 0.0f, 0.0f), float radius = 1.0f) : Collider(pos, mat3(1.0f)), m_r(radius) {};

    vec3 support(vec3 dirW){
        return normalize(dirW)*m_r + m_pos;
    }
};

//a Point3D is a sphere with tiny radius
//can be used with GJK
struct Point3D : Sphere {
    Point3D( vec3 pos = vec3(0.0f, 0.0f, 0.0f) ) : Sphere(pos, EPS) {}
    pluecker_point pluecker() { return { m_pos, 1}; };
};

//a 0D point - do not use in GJK
struct Point : Collider {
    Point( vec3 p ) : Collider(p) {}
    Point & operator=(const Point & l) = default;
    pluecker_point plueckerW() { return { m_pos, 1}; };

    vec3 support(vec3 dirW) {
        return m_pos; 
    }
};

//Cylinder: Height-aligned with y-axis (rotate using matRS)
struct Cylinder : Collider {
    float m_r, m_y_base, m_y_cap;

    vec3 support(vec3 dirW){
        auto dirL = dirW2L(dirW); //find support in model space
        vec3 dir_xz = vec3(dirL.x, 0, dirL.z);
        vec3 result = normalize(dir_xz)*m_r;
        result.y = (dirL.y>0) ? m_y_cap : m_y_base;
        return posL2W(result); //convert support to world space
    }
};

//Capsule: Height-aligned with y-axis
struct Capsule : Collider {
    float m_r, m_y_base, m_y_cap;

    Capsule(    vec3 pos = vec3{0.0f, 0.0f, 0.0f}, mat3 matRS = mat3(1.0f)
            ,   float radius = 0.2f, float yb = -0.5f, float yc = 0.5 ) 
                    : Collider( pos, matRS ), m_r(radius), m_y_base(yb), m_y_cap(yc)  {}

    vec3 support(vec3 dirW){
        auto dirL = dirW2L(dirW); //find support in model space
        vec3 result = normalize(dirL)*m_r;
        result.y += (dirL.y>0) ? m_y_cap : m_y_base;
        return posL2W(result); //convert support to world space
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

    float t( vec3 point ) {     //point = pos + t dir
        vec3 d = point - m_pos;
        float t0;
        if( std::abs(m_dir.x)>std::abs(m_dir.y) && std::abs(m_dir.x)>std::abs(m_dir.z)) {
            return d.x / m_dir.x;
        } else if( std::abs(m_dir.y)>std::abs(m_dir.z)) {
            return d.y / m_dir.y;
        }
        return d.z / m_dir.z;
    }

    vec3 support(vec3 dirW) {
        auto dirL = dirW2L(dirW); //find support in model space
        vec3 result = dot( dirL, m_dir ) < 0.0f ?  vec3{0,0,0} : m_dir;
        return posL2W(result); //convert support to world space
    }
};

//--------------------------------------------------------------------------------------

struct Polytope; 

struct PolytopePart : ICollider {
    PolytopePart() : ICollider() {};
};

struct VertexData {
    int  m_index;       //index of this vertex
    vint m_neighbors;   //indices of all vertex neighbors of this vertex

    VertexData(int i, vint& ne) : m_index(i), m_neighbors(ne) {}
    VertexData(int i, vint&& ne) : m_index(i), m_neighbors(ne) {}
};

//Vertex of a polytope
struct Vertex : PolytopePart {
    protected:
    Polytope*           m_polytope;
    const VertexData*   m_data;
    vec3                m_pointL;

    public:
    Vertex(Polytope* p, const VertexData *d, vec3 pointL ) : PolytopePart(), m_polytope(p), m_data(d), m_pointL(pointL) {}
    
    Polytope*       polytope() const { return m_polytope; };
    vec3 &          pointL() { return m_pointL; };
    vec3            pointW();
    const vint &    neighbors() const { assert(m_data!=nullptr); return m_data->m_neighbors;};
    pluecker_point  plueckerW() { return { pointW(), 1.0f}; };
    vec3            support(vec3 dir) { return pointW(); }
};

struct FaceData {
    int  m_index;
    vint m_vertices;          //indices of all vertices of this face
    vint m_neighbors;         //indices of the faces that are neighbors of this face

    FaceData(int i, vint& v, vint& ne )    : m_index(i), m_vertices(v), m_neighbors(ne) {}
    FaceData(int i, vint&& v, vint&& ne ) : m_index(i), m_vertices(v), m_neighbors(ne) {}
};

//Face of a polytope
struct Face  : PolytopePart {
    protected:
    Polytope*        m_polytope;
    const FaceData * m_data;

    public:
    Face(Polytope* p, const FaceData *d) : PolytopePart(), m_polytope(p), m_data(d) {}

    bool contains_vertex(int v ) const {
        auto &dm = m_data->m_vertices;
        return std::find( std::begin(dm), std::end(dm), v) != std::end(dm);
    }

    Polytope*       polytope() const { return m_polytope; };
    const vint &    vertices() const { return m_data->m_vertices; }
    const vint &    neighbors() const { return m_data->m_neighbors; }
    vec3            normalW() const;
    void            pointsW( vvec3 &points ) const;
    void            edgesW( std::vector<Line> & edges, std::set<ipair> *pairs = nullptr ) const;
    pluecker_plane  plueckerW() const;
    vec3            support(vec3 dir);
};

//Polytope: Just a set of points plus adjencency information
struct Polytope : Collider {
    protected:
    std::vector<Vertex>   m_vertices;
    std::vector<Face>     m_faces;
    int                   m_supporting_point = -1;

    public:
    Polytope( vec3 pos = {0,0,0}, mat3 matRS = mat3(1.0f) ) : Collider(pos, matRS) {}

    Polytope( vvec3& points, vec3 pos = {0,0,0}, mat3 matRS = mat3(1.0f) ) : Collider(pos, matRS) {
        for( const auto& p : points ) m_vertices.emplace_back(  this, nullptr, p );
    }

    Polytope(     const vvec3& points
                , const std::vector<VertexData>& vertex_data
                , const std::vector<FaceData>& face_data 
                , vec3 pos = {0,0,0}
                , mat3 matRS = mat3(1.0f) )
                     : Collider(pos, matRS) {
        int i=0;
        for( auto &v : vertex_data ) m_vertices.emplace_back( this, &v, points[i++] );
        for( auto &f : face_data )   m_faces.emplace_back( this, &f );
    }

    std::vector<Vertex>&   vertices() { return m_vertices; };
    std::vector<Face>&     faces() { return m_faces; };
    Vertex & vertex( int v) { return m_vertices[v]; }
    Face &   face( int f ) { return m_faces[f]; }

    //return the edges of this polytope, include each edge only once
    void edgesW( std::vector<Line> & edges ) {
        std::set<ipair> pairs;
        for( auto & face : m_faces ) {
            face.edgesW( edges, &pairs );
        }
    }

    vec3 support(vec3 dirW) {
        if( m_vertices.empty()) return {0,0,0};

        auto dirL = dirW2L(dirW);
        vec3 furthest_point; 
        float max_dot;

        if( m_vertices[0].neighbors().empty() ) {
            furthest_point = m_vertices[0].pointL();
            max_dot = dot(furthest_point, dirL);
            std::for_each(  std::begin(m_vertices), std::end(m_vertices), 
                            [&]( auto &vertex) {
                                float d = dot(vertex.pointL(), dirL);
                                if(d>max_dot){
                                    max_dot = d;
                                    furthest_point = vertex.pointL();
                                }
                            } );
        } else {
            //ADD YOUR CODE HERE TO ITERATE THROUGH NEIGHBORS rather than iterate through all points
            m_supporting_point = m_supporting_point>=0 ? m_supporting_point : 0;
            furthest_point = m_vertices[m_supporting_point].pointL(); 
            max_dot = dot(furthest_point, dirL);
            int maxi = m_supporting_point;

            do {
                m_supporting_point = maxi;
                for( auto& nei : m_vertices[m_supporting_point].neighbors() ) {
                    float d = dot(m_vertices[nei].pointL(), dirL);
                    if(d>max_dot){
                        max_dot = d;
                        furthest_point = m_vertices[nei].pointL();
                        maxi = nei;
                    }
                }
            } while( maxi != m_supporting_point );
        }

        return posL2W(furthest_point);
    }
};

struct Tetrahedron : Polytope {

    const static inline std::vector<VertexData> m_vertices_data =  //4 vertices, each is member of 3 faces and has 3 neighbors
                        {       VertexData{ 0, {1,2,3} }  //0
							, 	VertexData{ 1, {0,2,3} }  //1
							, 	VertexData{ 2, {0,1,3} }  //2
							, 	VertexData{ 3, {0,1,2} }  //3
                        };

	const static inline std::vector<FaceData> m_faces_data =  //4 faces, each has 3 vertices (clockwise), 3 neighbor faces
                    {       FaceData{ 0, {0,1,2}, {1,2,3} }  //0
                        ,   FaceData{ 1, {0,3,1}, {0,2,3} }  //1
                        ,   FaceData{ 2, {1,3,2}, {0,1,3} }  //2
                        ,   FaceData{ 3, {2,3,0}, {0,1,2} }  //3
                    };

    const static inline vvec3 m_points_data = {{-1,0,0},{1,0,0},{0,0,1},{0,1,0.5}};

    Tetrahedron()  : Polytope( m_points_data, m_vertices_data, m_faces_data ) {}

    Tetrahedron( vec3 p0, vec3 p1, vec3 p2, vec3 p3 )  : Polytope( m_points_data, m_vertices_data, m_faces_data ) {
        m_vertices[0].pointL() = p0;
        m_vertices[1].pointL() = p1;
        m_vertices[2].pointL() = p2;
        m_vertices[3].pointL() = p3;
    };
};

//a triangle is a tertrahedron with tiny height
//can be used with GJK
struct Triangle3D : Tetrahedron {
    Triangle3D( vec3 p0, vec3 p1, vec3 p2 )  : Tetrahedron( p0, p1, p2, {0,0,0} ) {
        vec3 d0 = p2 - p0;
        vec3 d1 = p1 - p0;
        vec3 up = EPS * normalize( cross( d0,  d1) );
        m_vertices[3].pointL() = (p0 + p1 + p2)*0.33333f + up;
    };
};

//a box is a polytope with 8 vertices
struct Box : Polytope {
                                //every vertex has 3 neighbors
    const static inline std::vector<VertexData> m_vertices_data = 
                        { 
                                VertexData{ 0, {1,2,4} }   // 0  
							, 	VertexData{ 1, {0,3,5} }   // 1
							, 	VertexData{ 2, {0,3,6} }   // 2
							, 	VertexData{ 3, {1,2,7} }   // 3
                            ,   VertexData{ 4, {0,5,6} }   // 4
							, 	VertexData{ 5, {1,4,7} }   // 5
							, 	VertexData{ 6, {2,4,7} }   // 6
							, 	VertexData{ 7, {3,5,6} }   // 7
						};

                        //6 faces, each having 4 vertices (clockwise), 4 neighbor faces
    const static inline std::vector<FaceData> m_faces_data =  
                    {   FaceData{ 0, {0,2,4,6}, {2,3,4,5} }   //0
                    ,   FaceData{ 1, {1,3,5,7}, {2,3,4,5} }   //1
                    ,   FaceData{ 2, {0,1,2,3}, {0,1,4,5} }   //2
                    ,   FaceData{ 3, {4,5,6,7}, {0,1,4,5} }   //3
                    ,   FaceData{ 4, {0,1,4,5}, {0,1,2,3} }   //4
                    ,   FaceData{ 5, {2,3,6,7}, {0,1,2,3} }   //5
                    };

    const static inline vvec3 m_points_data = //3D coordinates in local space
                    {   vec3(-0.5f, -0.5f, -0.5f), vec3(0.5f, -0.5f, -0.5f)
                    ,   vec3(-0.5f, -0.5f, 0.5f),  vec3(0.5f, -0.5f, 0.5f)
                    ,   vec3(-0.5f,  0.5f, -0.5f), vec3(0.5f,  0.5f, -0.5f)
                    ,   vec3(-0.5f,  0.5f, 0.5f),  vec3(0.5f,  0.5f, 0.5f)
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
        vvec3 points = {p0, p1, p2, p3, p0 + up, p1 + up, p2 + up, p3 + up};
        int i = 0;
        for( auto& vertex : m_vertices ) vertex.pointL() = points[i++];
    };
};

//a Polygon3D with N vertices, all vertices must lie in the same plane
//the polgon has a 3D body but is infinitely thin
//can be used with GJK
struct Polygon3D : Polytope {
    Polygon3D( vvec3& points ) : Polytope() {
        vec3 d0 = points[2] - points[0];
        vec3 d1 = points[1] - points[0];
        vec3 up = EPS * normalize( cross( d0,  d1) );

        for( auto &p : points ) m_vertices.emplace_back( this, nullptr, p );
        for( auto &p : points ) m_vertices.emplace_back( this, nullptr, p + up );
    }
};

//-------------------------------------------------------------------------------------

//Polytope parts

//return the position of a vertex in world coordinates
vec3 Vertex::pointW() {
    return m_polytope->matRS() * pointL() + m_polytope->pos() ;
}

//get the normal of the face
vec3 Face::normalW() const {
    Vertex v0 = m_polytope->vertices()[0];
    Vertex v1 = m_polytope->vertices()[1];
    Vertex v2 = m_polytope->vertices().back();
    return cross( v1.pointW() - v0.pointW(), v2.pointW() - v0.pointW());
}

//return a list with the coordinates of the vertices of a given face in world coordinates
void Face::pointsW( vvec3 &points ) const {
    for( auto i : vertices()) {
        points.push_back( m_polytope->vertices()[i].pointW() );
    }
}

//return a list of edges of this face
//if pairs ppoints to a set, make sure that each edge is included only once
void Face::edgesW( std::vector<Line> & edges, std::set<ipair> *pairs  ) const {
    int v0 = vertices().back();
    for( int v : vertices() ) {
        if( pairs==nullptr || !pairs->contains(std::make_pair(v0, v)) ) {
            edges.emplace_back( m_polytope->vertices()[v0].pointW(), m_polytope->vertices()[v].pointW() );
            if( pairs!=nullptr) pairs->insert(std::make_pair(v, v0)); //do not include this edge a 2nd time with neighbor face
        }
        v0 = v;
    }
}

//returns a pluecker plane from the face
pluecker_plane Face::plueckerW() const {
    vec3 q = m_polytope->vertices()[vertices()[0]].pointW();
    vec3 normal = normalW();
    return { normal, -1.0f * dot( normal, q)};
}

//suport function for the face
vec3 Face::support(vec3 dirW) {
    auto dirL = m_polytope->dirW2L(dirW); //find support in model space
    vec3 maxL = m_polytope->vertices()[0].pointL();
    float max = dot( dirL, maxL );
    for( auto i : m_data->m_vertices ) {
        float d = dot( dirL, m_polytope->vertices()[i].pointL() );
        if( d > max ) {
            maxL = m_polytope->vertices()[i].pointL();
            max = d;
        }
    }
    return m_polytope->posL2W(maxL); //convert support to world space
}




