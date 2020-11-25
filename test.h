#ifndef TEST_H
#define TEST_H

#include <iterator>

#include "gjk_epa.h"
#include "collider.h"
#include "collision.h"
#include "contact.h"
#include "distance.h"



namespace vpe {

	inline bool unit_sub_test( bool result ) {
		if( result ) {
			return true;			
		}
		std::cout << " -- FAIL\n";
		std::exit(1);
	}

	inline bool unit_test_pluecker() {
		{
			Line l1( {-1,1,0}, {1,1,0} );
			Line l2( {0,0,-1}, {0,0,1} );
			float d = distance_line_line( l1.plueckerW(), l2.plueckerW());
			if( d!=1) return false;
		}
		{
			Line l1( {-1,0,0}, {1,0,0} );
			Line l2( {0,0,-1}, {0,0,1} );
			float d = distance_line_line( l1.plueckerW(), l2.plueckerW());
			if( d!=0) return false;
		}
		{
			pluecker_line l1( {-1,0,0}, {1,0,0} );
			pluecker_plane p1( {1,0,0}, {2,0,0} );
			auto point = intersect_line_plane( l1, p1);
			if( point.p3D()!=vec3{2,0,0}) return false;			
		}
		return true;
	}


    inline bool unit_test_normals() {
	    Box box{ {0.0f, 0.5f, 0.0f} };
		vvec3 normals = {
			box.face(0).normalW(), box.face(1).normalW(),
			box.face(2).normalW(), box.face(3).normalW(),
			box.face(4).normalW(), box.face(5).normalW()};
		
		vvec3 normals2 = {{-1,0,0},{1,0,0},{0,-1,0},{0,1,0},{0,0,-1},{0,0,1}};
		if( !std::equal(std::begin(normals), std::end(normals), std::begin(normals2) ) ) return false;

        return true;
    }

    inline bool unit_test_box_box() {
		std::cout << "Box-Box tests\n";
		{
			std::cout << "Face-Face 1";
			Box ground{ {0.0f, -50.0f, 0.0f}, scale( mat4(1.0f), vec3(100.0f, 100.0f, 100.0f)) };
			Box box{ {0.0f, 0.41f, 0.0f} };
			vec3 mtv(0,1,0); //minimum translation vector
			auto hit = gjk( box, ground, mtv);
			if(!hit) return false;
			if( mtv != vec3{ 0, 0.09f, 0 }) return false;

			box.pos() += mtv;
			std::set<contact> ct;
			contacts( box, ground, mtv, ct);
			if( ct.size()!=4) return false;
			std::cout << " -- OK\n";
		}

		{
			std::cout << "Face-Face 2";
			Box box1{ {0.0f, 1.0f, 0.0f}, scale( mat4(1.0f), vec3(2.0f, 2.0f, 2.0f)) };
			Box box2{ {2.0f, 1.0f, 0.0f}, scale( mat4(1.0f), vec3(2.0f, 2.0f, 2.0f)) };
			vec3 mtv(0,0,0); //minimum translation vector
			auto hit = gjk( box1, box2, mtv);
			if( !hit ) return false;
			if( mtv != vec3{ 0, 0.0f, 0 }) return false;

			box1.pos() += mtv;
			std::set<contact> ct;
			contacts( box1, box2, mtv, ct);
			if( ct.size()!=4) return false;
			std::cout << " -- OK\n";
		}

		{
			std::cout << "Face-Face 3";
			Box box1{ {0.1f, 1.9f, 0.1f} };
			Box box2{ {0.0f, 1.0f, 0.0f} }; //, rotate( mat4(1.0f), (float)(M_PI / 4.0f), vec3{0.0f,0.0f,1.0f} ) };
			vec3 mtv(0,0,0); //minimum translation vector
			auto hit = gjk( box1, box2, mtv);
			if( !hit ) return false;
			if( mtv != vec3{ 0, 0.1f, 0 }) return false;

			box1.pos() += mtv;
			std::set<contact> ct;
			contacts( box1, box2, mtv, ct);
			if( ct.size()!=4) return false;
			std::cout << " -- OK\n";
		}

		return true;
	}



    inline bool unit_tests() {
		unit_sub_test( unit_test_pluecker() );
		unit_sub_test( unit_test_normals() );
		unit_sub_test( unit_test_box_box() );
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

