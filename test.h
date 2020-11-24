#ifndef TEST_H
#define TEST_H


#include "gjk_epa.h"
#include "collider.h"
#include "collision.h"
#include "contact.h"



namespace vpe {

    inline bool unit_test_normals() {
	    Box box{ {0.0f, 0.41f, 0.0f} };
        vec3 n0 = box.face(0).normalW();
	    vec3 n1 = box.face(1).normalW();
	    vec3 n2 = box.face(2).normalW();
	    vec3 n3 = box.face(3).normalW();
	    vec3 n4 = box.face(4).normalW();
	    vec3 n5 = box.face(5).normalW();
        return true;
    }

    inline bool unit_test_face_face_collision() {
	    Box ground{ {0.0f, -50.0f, 0.0f}, scale( mat4(1.0f), vec3(100.0f, 100.0f, 100.0f)) };
	    Box box{ {0.0f, 0.41f, 0.0f} };
        vec3 mtv(0,1,0); //minimum translation vector
	    auto hit1 = gjk( box, ground, mtv);
		return true;
	}

    inline bool unit_test_edge_edge_collision() {
	    Box box1{ {0.0f, 1.0f, 0.0f}, scale( mat4(1.0f), vec3(2.0f, 2.0f, 2.0f)) };
	    Box box2{ {2.0f, 1.0f, 0.0f}, scale( mat4(1.0f), vec3(2.0f, 2.0f, 2.0f)) };
        vec3 mtv(0,0,0); //minimum translation vector
	    auto hit1 = gjk( box1, box2, mtv);
        return true;
	}

    inline bool unit_test_collision() {
		unit_test_face_face_collision();
		unit_test_edge_edge_collision();
        return true;
    }

    inline bool unit_test_contacts() {
	    Box ground{ {0.0f, -50.0f, 0.0f}, scale( mat4(1.0f), vec3(100.0f, 100.0f, 100.0f)) };
	    Box box{ {0.0f, 0.41f, 0.0f} };
        vec3 mtv(0,1,0); //minimum translation vector
	    auto hit1 = gjk( box, ground, mtv);

	    box.pos() += mtv;
	    std::set<contact> ct;
	    contacts( box, ground, mtv, ct);
        return true;
    }


    bool unit_test_triangles() {
        Line3D line{ {0.f,0.f, -1.f}, {0.f,0.0f,1.f} };

	    Tetrahedron tet( {-1.0f, 0.0f, -1.0f}, {1.0f, 0.0f, -1.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 1.0f, 0.5f}  );
	    Tetrahedron tet2{};

	    Triangle3D tri{ {-1.0f, 0.0f, -1.0f}, {1.0f, 0.0f, -1.0f}, {0.0f, 0.0f, 1.0f} };

	    Point3D point{ {0,-10,0} };
	    //auto hit3 = gjk( point, ground, mtv);

	    Triangle3D triangle{ {-1,0,0}, {1,0,0}, {0,1,0} };
	    //auto hit4 = gjk( triangle, ground, mtv);

	    Quad3D quad{ {-1,0,-1}, {1,0,-1}, {1,0,1}, {-1,0,1} };
        return true;
    }


}


#endif

