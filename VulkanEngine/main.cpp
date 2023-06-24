/**
* The Vienna Vulkan Engine
*
* (c) bei Helmut Hlavacs, University of Vienna, 2022
*
*/

#include <algorithm>
#include <vector>
#include <cstdio>
#include <iterator>
#include <ranges>
#include <string>
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <random>

#include "VEInclude.h"

#define GLM_ENABLE_EXPERIMENTAL
#define GLM_FORCE_LEFT_HANDED
#include "glm/glm.hpp"
#include "glm/gtx/matrix_operation.hpp"
#include "glm/gtc/quaternion.hpp"
#include "glm/gtx/quaternion.hpp"
#include "glm/gtx/matrix_cross_product.hpp"

#include "VPE.hpp"

using namespace vpe;


namespace ve {

	//---------------------------------------------------------------------------------------------------------
	//callbacks for bodies

	/// <summary>
	/// This callback is used for updating the visual body whenever a physics body moves.
	/// It is also used for extrapolating the new position between two simulation slots.
	/// </summary>
	inline VPEWorld::callback_move onMove = [&](double dt, std::shared_ptr<VPEWorld::Body> body) {
		VESceneNode* cube = static_cast<VESceneNode*>(body->m_owner);		//Owner is a pointer to a scene node
		glmvec3 pos = body->m_positionW;									//New position of the scene node
		glmquat orient = body->m_orientationLW;								//New orientation of the scende node
		body->stepPosition(dt, pos, orient, false);							//Extrapolate
		cube->setTransform(VPEWorld::Body::computeModel(pos, orient, body->m_scale));	//Set the scene node data
	};

	/// <summary>
	/// This callback is called if the body is intentionally deleted. It is not called if the engine shuts down
	/// and all bodies are deleted.
	/// </summary>
	inline VPEWorld::callback_erase onErase = [&](std::shared_ptr<VPEWorld::Body> body) {
		VESceneNode* node = static_cast<VESceneNode*>(body->m_owner);		//Owner is a pointer to a scene node
		getSceneManagerPointer()->deleteSceneNodeAndChildren(((VESceneNode*)body->m_owner)->getName());
	};

	/// <summary>
	/// This is an example callback that is called if a body collides with another body.
	/// The first parameter is the body for which this collides was set, the second body
	/// is the body that collided with it.
	/// </summary>
	inline VPEWorld::callback_collide onCollide =
		[](std::shared_ptr<VPEWorld::Body> body1, std::shared_ptr<VPEWorld::Body> body2) {
		std::cout << "Collision " << body1->m_name << " " << body2->m_name << "\n";
	};


	//---------------------------------------------------------------------------------------------------------
	//Listener for driving the simulation 

	class VEEventListenerPhysics : public VEEventListener {
	protected:

		/// <summary>
		/// This drives the simulation!!!
		/// </summary>
		void onFrameStarted(veEvent event) {
			m_physics->tick(event.dt);
		}

		VPEWorld* m_physics;	//Pointer to the physics world

	public:
		///Constructor of class EventListenerCollision
		VEEventListenerPhysics(std::string name, VPEWorld* physics) : VEEventListener(name), m_physics{ physics } { };

		///Destructor of class EventListenerCollision
		virtual ~VEEventListenerPhysics() {};
	};

	 /// <summary>
	 /// Creates a wheel-like structure with one center cube at init_pos with num_cubes rotated equally around it
	 /// </summary>
	 /// <param name="init_pos"></param>
	 /// <param name="num_cubes"></param>
	 /// <returns>Returns a pointer to the center cube's body</returns>
	std::shared_ptr<VPEWorld::Body> createWheel(glmvec3 init_pos, int num_cubes, vpe::VPEWorld* m_physics) {
		real cube_mass = 1.0_real / 100.0_real;
		real center_mass = 0.0_real;
		center_mass = cube_mass;

		VESceneNode* centerCube;
		VECHECKPOINTER(centerCube = getSceneManagerPointer()->loadModel("The Cube" + std::to_string(m_physics->m_body_id), "media/models/test/crate0", "cube.obj", 0, getRoot()));
		auto centerBody = std::make_shared<VPEWorld::Body>(m_physics, "Body" + std::to_string(m_physics->m_bodies.size()), centerCube, &m_physics->g_cube, glmvec3{ 1.0_real }, init_pos, glmquat{ 1,0,0,0 }, glmvec3{ 0.0_real }, glmvec3{ 0.0_real }, center_mass, m_physics->m_restitution, m_physics->m_friction);
		centerBody->m_on_move = onMove;
		centerBody->m_on_erase = onErase;
		m_physics->addBody(centerBody);

		glmvec3 pos = init_pos;
		pos[1] += 3.0_real;
		std::vector<VESceneNode*> cubes;
		std::vector<std::shared_ptr<VPEWorld::Body>> bodies;

		glmmat4 rot(1.0_real);
		real rot_amount = pi2 / (num_cubes);

		for (int i = 0; i < num_cubes; ++i) {
			VESceneNode* cube;
			VECHECKPOINTER(cube = getSceneManagerPointer()->loadModel("The Cube" + std::to_string(m_physics->m_body_id), "media/models/test/crate0", "cube.obj", 0, getRoot()));
			cubes.push_back(cube);
			auto body = std::make_shared<VPEWorld::Body>(m_physics, "Body" + std::to_string(m_physics->m_bodies.size()), cube, &m_physics->g_cube, glmvec3{ 1.0_real }, pos, glmquat{ 1,0,0,0 }, glmvec3{ 0.0_real }, glmvec3{ 0.0_real }, cube_mass, m_physics->m_restitution, m_physics->m_friction);
			bodies.push_back(body);
			body->m_on_move = onMove;
			body->m_on_erase = onErase;
			m_physics->addBody(body);
			body->setForce(0ul, VPEWorld::Force{ {0, m_physics->c_gravity, 0} });

			pos -= init_pos;
			pos = glm::rotate(rot_amount, glmvec3(0, 0, 1)) * glmvec4(pos, 1.0_real);
			pos += init_pos;
		}

		for (int i = 0; i < num_cubes; ++i) {
			auto constraint = std::make_shared<VPEWorld::FixedJoint>(centerBody, bodies[i], init_pos);
			m_physics->addConstraint(constraint);
		}

		return centerBody;
	}


	//---------------------------------------------------------------------------------------------------------
	//Listener for creating bodies with keyboard

	/// <summary>
	/// This is a callback that is called in each loop. It implements a simple rigid body 
	/// physics engine.
	/// </summary>
	class VEEventListenerPhysicsKeys : public VEEventListener {

		std::default_random_engine rnd_gen{ 12345 };					//Random numbers
		std::uniform_real_distribution<> rnd_unif{ 0.0f, 1.0f };		//Random numbers

	public:

		/// <summary>
		/// Callback for event key stroke. Depending on the key pressed, bodies are created.
		/// </summary>
		/// <param name="event">The keyboard event.</param>
		/// <returns>False, so the key is not consumed.</returns>
		bool onKeyboard(veEvent event) {

			if (event.idata1 == GLFW_KEY_B && event.idata3 == GLFW_PRESS) {
				glmvec3 positionCamera{ getSceneManagerPointer()->getSceneNode("StandardCameraParent")->getWorldTransform()[3] };
				glmvec3 dir{ getSceneManagerPointer()->getSceneNode("StandardCamera")->getWorldTransform()[2] };
				glmvec3 vel = (30.0_real + 5.0_real * (real)rnd_unif(rnd_gen)) * dir / glm::length(dir);
				glmvec3 scale{ 1,1,1 }; // = rnd_unif(rnd_gen) * 10;
				real angle = (real)rnd_unif(rnd_gen) * 10 * 3 * (real)M_PI / 180.0_real;
				glmvec3 orient{ rnd_unif(rnd_gen), rnd_unif(rnd_gen), rnd_unif(rnd_gen) };
				glmvec3 vrot{ rnd_unif(rnd_gen) * 5, rnd_unif(rnd_gen) * 5, rnd_unif(rnd_gen) * 5 };
				VESceneNode* cube;
				VECHECKPOINTER(cube = getSceneManagerPointer()->loadModel("The Cube" + std::to_string(m_physics->m_body_id), "media/models/test/crate0", "cube.obj", 0, getRoot()));
				auto body = std::make_shared<VPEWorld::Body>( m_physics, "Body" + std::to_string(m_physics->m_bodies.size()), cube, & m_physics->g_cube, scale, positionCamera + 2.0_real * dir, glm::rotate(angle, glm::normalize(orient)), vel, vrot, 1.0_real / 100.0_real, m_physics->m_restitution, m_physics->m_friction);
				body->setForce( 0ul, VPEWorld::Force{ {0, m_physics->c_gravity, 0} } );
				body->m_on_move = onMove;
				body->m_on_erase = onErase;
				m_physics->addBody(body);
			}

			if (event.idata1 == GLFW_KEY_SPACE && event.idata3 == GLFW_PRESS) {
				glmvec3 positionCamera{ getSceneManagerPointer()->getSceneNode("StandardCameraParent")->getWorldTransform()[3] };

				for (int i = 0; i < 1; ++i) {
					VESceneNode* cube0;
					static real dy = 0.5_real;
					VECHECKPOINTER(cube0 = getSceneManagerPointer()->loadModel("The Cube" + std::to_string(m_physics->m_body_id), "media/models/test/crate0", "cube.obj", 0, getRoot()));
					auto body = std::make_shared<VPEWorld::Body>(m_physics, "Body" + std::to_string(m_physics->m_bodies.size()), cube0, &m_physics->g_cube, glmvec3{ 1.0_real }, glmvec3{positionCamera.x, dy++, positionCamera.z + 4}, glmquat{ 1,0,0,0 }, glmvec3{0.0_real}, glmvec3{0.0_real}, 1.0_real / 100.0_real, m_physics->m_restitution, m_physics->m_friction );
					body->setForce( 0ul, VPEWorld::Force{ {0, m_physics->c_gravity, 0} } );
					body->m_on_move = onMove;
					body->m_on_erase = onErase;
					m_physics->addBody(body);
				}
			}

			if (event.idata1 == GLFW_KEY_Y && event.idata3 == GLFW_PRESS) {
				glmvec3 positionCamera{ getSceneManagerPointer()->getSceneNode("StandardCameraParent")->getWorldTransform()[3] };

				for (int dy = 0; dy < 15; ++dy) {
					for (int dx = 0; dx < 15 - dy; ++dx) {
						VESceneNode* cube0;
						VECHECKPOINTER(cube0 = getSceneManagerPointer()->loadModel("The Cube" + std::to_string(m_physics->m_body_id), "media/models/test/crate0", "cube.obj", 0, getRoot()));
						auto body = std::make_shared<VPEWorld::Body>(m_physics, "Body" + std::to_string(m_physics->m_bodies.size()), cube0, &m_physics->g_cube, glmvec3{1.0_real}, glmvec3{dx + 0.4 * dy, 0.5_real + dy, 0.0_real}, glmquat{1,0,0,0}, glmvec3{ 0.0_real }, glmvec3{ 0.0_real }, 1.0_real / 100.0_real, m_physics->m_restitution, m_physics->m_friction);
						body->setForce( 0ul, VPEWorld::Force{ {0, m_physics->c_gravity, 0} } );
						body->m_on_move = onMove;
						body->m_on_erase = onErase;
						m_physics->addBody(body);
					}
				}
			}

			if (event.idata1 == GLFW_KEY_Z && event.idata3 == GLFW_PRESS) {
				glmvec3 positionCamera{ getSceneManagerPointer()->getSceneNode("StandardCameraParent")->getWorldTransform()[3] };
				glmvec3 dir{ getSceneManagerPointer()->getSceneNode("StandardCamera")->getWorldTransform()[2] };
				VESceneNode* cube0;
				VECHECKPOINTER(cube0 = getSceneManagerPointer()->loadModel("The Cube" + std::to_string(m_physics->m_body_id), "media/models/test/crate0", "cube.obj", 0, getRoot()));
				auto body = std::make_shared<VPEWorld::Body>(m_physics, "Body" + std::to_string(m_physics->m_bodies.size()), cube0, &m_physics->g_cube, glmvec3{1.0_real}, positionCamera + 2.0_real * dir, glmquat{1,0,0,0}, glmvec3{0.0_real}, glmvec3{0.0_real}, 1.0_real / 100.0_real, m_physics->m_restitution, m_physics->m_friction );
				body->setForce( 0ul, VPEWorld::Force{ {0, m_physics->c_gravity, 0} } );
				body->m_on_move = onMove;
				body->m_on_erase = onErase;
				m_physics->addBody(body);
			}

			if (event.idata1 == GLFW_KEY_C && event.idata3 == GLFW_PRESS) {
				glmvec3 positionCamera{ getSceneManagerPointer()->getSceneNode("StandardCameraParent")->getWorldTransform()[3] };
				glmvec3 dir{ getSceneManagerPointer()->getSceneNode("StandardCamera")->getWorldTransform()[2] };

				real cubeMass = 0;
				std::shared_ptr<VPEWorld::Body> prevBody;
				positionCamera[1] += 8.0_real;

				for (int i = 0; i < 10; ++i) {
					if (i == 0 || i == 9) cubeMass = 0;
					else cubeMass = 1.0_real / 100.0_real;

					VESceneNode* cube;
					VECHECKPOINTER(cube = getSceneManagerPointer()->loadModel("The Cube" + std::to_string(m_physics->m_body_id), "media/models/test/crate0", "cube.obj", 0, getRoot()));
					auto body = std::make_shared<VPEWorld::Body>(m_physics, "Body" + std::to_string(m_physics->m_bodies.size()), cube, &m_physics->g_cube, glmvec3{ 1.0_real }, positionCamera + 2.0_real * dir, glmquat{ 1,0,0,0 }, glmvec3{ 0.0_real }, glmvec3{ 0.0_real }, cubeMass, m_physics->m_restitution, m_physics->m_friction);
				
					if (i != 0 && i != 9) {
						body->setForce(0ul, VPEWorld::Force{ {0, m_physics->c_gravity, 0} });
					}

					body->m_on_move = onMove;
					body->m_on_erase = onErase;
					m_physics->addBody(body);

					if (i > 0) {
						auto constraint = std::make_shared<VPEWorld::DistanceConstraint>(body, prevBody, 2.3_real);
						m_physics->addConstraint(constraint);
					}

					prevBody = body;
					positionCamera[0] += 2.0_real;
				}
			}

			// TODO Maybe: Try combining distance + socket constraints (make limbs with distance -> connect in joints)
			if (event.idata1 == GLFW_KEY_M && event.idata3 == GLFW_PRESS) {
				glmvec3 positionCamera{ getSceneManagerPointer()->getSceneNode("StandardCameraParent")->getWorldTransform()[3] };
				glmvec3 dir{ getSceneManagerPointer()->getSceneNode("StandardCamera")->getWorldTransform()[2] };
				positionCamera[1] += 3.0_real;

				glmvec3 cubePos1 = positionCamera + 2.0_real * dir;
				glmvec3 cubePos2 = cubePos1;
				cubePos2[0] += 2.0;
				//glmvec3 jointAnchor = 0.5_real * (cubePos1 + cubePos2) - glmvec3(0.0_real, 0.5_real, 0.0_real);
				glmvec3 jointAnchor = cubePos1;

				VESceneNode* cube0;
				VECHECKPOINTER(cube0 = getSceneManagerPointer()->loadModel("The Cube" + std::to_string(m_physics->m_body_id), "media/models/test/crate0", "cube.obj", 0, getRoot()));
				auto body = std::make_shared<VPEWorld::Body>(m_physics, "Body" + std::to_string(m_physics->m_bodies.size()), cube0, &m_physics->g_cube, glmvec3{ 1.0_real }, cubePos1, glmquat{ 1,0,0,0 }, glmvec3{ 0.0_real }, glmvec3{ 0.0_real }, 0.0_real, m_physics->m_restitution, m_physics->m_friction);
				//	body->setForce(0ul, VPEWorld::Force{ {0, m_physics->c_gravity, 0} });
				body->m_on_move = onMove;
				body->m_on_erase = onErase;
				m_physics->addBody(body);

				VESceneNode* cube1;
				VECHECKPOINTER(cube1 = getSceneManagerPointer()->loadModel("The Cube" + std::to_string(m_physics->m_body_id), "media/models/test/crate0", "cube.obj", 0, getRoot()));
				auto body1 = std::make_shared<VPEWorld::Body>(m_physics, "Body" + std::to_string(m_physics->m_bodies.size()), cube1, &m_physics->g_cube, glmvec3{ 1.0_real }, cubePos2, glmquat{ 1,0,0,0 }, glmvec3{ 0.0_real }, glmvec3{ 0.0_real }, 1.0_real / 100.0_real, m_physics->m_restitution, m_physics->m_friction);
				body1->m_on_move = onMove;
				body1->m_on_erase = onErase;
				body1->setForce(0ul, VPEWorld::Force{ {0, m_physics->c_gravity, 0} });
				m_physics->addBody(body1);

				auto constraint = std::make_shared<VPEWorld::BallSocketJoint>(body, body1, jointAnchor);
				m_physics->addConstraint(constraint);
				
			}

			if (event.idata1 == GLFW_KEY_H && event.idata3 == GLFW_PRESS) {
				glmvec3 positionCamera{ getSceneManagerPointer()->getSceneNode("StandardCameraParent")->getWorldTransform()[3] };
				glmvec3 dir{ getSceneManagerPointer()->getSceneNode("StandardCamera")->getWorldTransform()[2] };
				positionCamera[1] += 2.0_real;

				glmvec3 cubePos1 = positionCamera + 2.0_real * dir;
				glmvec3 cubePos2 = cubePos1;
				cubePos2[0] += 2.0_real;
			//	glmvec3 jointAnchor = 0.5_real * (cubePos1 + cubePos2) - glmvec3(0.0_real, 0.5_real, 0.0_real);
			//	glmvec3 jointAnchor = 0.5_real * (cubePos1 + cubePos2);
				glmvec3 jointAnchor = cubePos1;
				glmvec3 jointAxis(0.0_real, 1.0_real, 0.0_real);

				VESceneNode* cube0;
				VECHECKPOINTER(cube0 = getSceneManagerPointer()->loadModel("The Cube" + std::to_string(m_physics->m_body_id), "media/models/test/crate0", "cube.obj", 0, getRoot()));
				auto body = std::make_shared<VPEWorld::Body>(m_physics, "Body" + std::to_string(m_physics->m_bodies.size()), cube0, &m_physics->g_cube, glmvec3{ 1.0_real }, cubePos1, glmquat{ 1,0,0,0 }, glmvec3{ 0.0_real }, glmvec3{ 0.0_real }, 0.0_real, m_physics->m_restitution, m_physics->m_friction);
				//body->setForce(0ul, VPEWorld::Force{ {0, m_physics->c_gravity, 0} });
				body->m_on_move = onMove;
				body->m_on_erase = onErase;
				m_physics->addBody(body);

				VESceneNode* cube1;
				VECHECKPOINTER(cube1 = getSceneManagerPointer()->loadModel("The Cube" + std::to_string(m_physics->m_body_id), "media/models/test/crate0", "cube.obj", 0, getRoot()));
				auto body1 = std::make_shared<VPEWorld::Body>(m_physics, "Body" + std::to_string(m_physics->m_bodies.size()), cube1, &m_physics->g_cube, glmvec3{ 1.0_real }, cubePos2, glmquat{ 1,0,0,0 }, glmvec3{ 0.0_real }, glmvec3{ 0.0_real }, 1.0_real / 100.0_real, m_physics->m_restitution, m_physics->m_friction);
				body1->m_on_move = onMove;
				body1->m_on_erase = onErase;
				body1->setForce(0ul, VPEWorld::Force{ {0, m_physics->c_gravity, 0} });
				m_physics->addBody(body1);

				auto constraint = std::make_shared<VPEWorld::HingeJoint>(body, body1, jointAnchor, jointAxis);
				//constraint->enableMotor(-3.0_real, 4.0_real);
				constraint->enableLimit(0, pi);
				m_physics->addConstraint(constraint);

			}

			if (event.idata1 == GLFW_KEY_N && event.idata3 == GLFW_PRESS) {
				glmvec3 positionCamera{ getSceneManagerPointer()->getSceneNode("StandardCameraParent")->getWorldTransform()[3] };
				glmvec3 dir{ getSceneManagerPointer()->getSceneNode("StandardCamera")->getWorldTransform()[2] };

				glmvec3 centerPos = positionCamera + 2.0_real * dir;
				centerPos[1] += 7.0_real;

				auto center1 = createWheel(centerPos, 6, m_physics);
				centerPos[2] += 6;
				auto center2 = createWheel(centerPos, 6, m_physics);

				centerPos = 0.5_real * (center1->m_positionW + center2->m_positionW);

				VESceneNode* centerCube;
				VECHECKPOINTER(centerCube = getSceneManagerPointer()->loadModel("The Cube" + std::to_string(m_physics->m_body_id), "media/models/test/crate0", "cube.obj", 0, getRoot()));
				auto centerBody = std::make_shared<VPEWorld::Body>(m_physics, "Body" + std::to_string(m_physics->m_bodies.size()), centerCube, &m_physics->g_cube, glmvec3{ 1.0_real }, centerPos, glmquat{ 1,0,0,0 }, glmvec3{ 0.0_real }, glmvec3{ 0.0_real }, 1.0_real / 100.0_real, m_physics->m_restitution, m_physics->m_friction);
				centerBody->m_on_move = onMove;
				centerBody->m_on_erase = onErase;
				m_physics->addBody(centerBody);

				real motor_speed = 3.0_real;
				real motor_max_force = 38.0_real;

				glmvec3 jointAxis{ 0.0_real, 0.0_real, 1.0_real };
				auto constraint1 = std::make_shared<VPEWorld::HingeJoint>(centerBody, center1, centerPos, jointAxis);
				m_physics->addConstraint(constraint1);
				constraint1->enableMotor(motor_speed, motor_max_force);
				constraint1->setBody1MotorEnabled(false);

				auto constraint2 = std::make_shared<VPEWorld::HingeJoint>(centerBody, center2, centerPos, jointAxis);
				m_physics->addConstraint(constraint2);
				constraint2->enableMotor(motor_speed, motor_max_force);
				constraint2->setBody1MotorEnabled(false);

				std::thread flipMotorThread([constraint1, constraint2, motor_speed, motor_max_force]() {
					int sign = 1;
					// Create this here as well, otherwise it will be undefined when the function is done but the thread is still running
					real m_motor_speed = motor_speed;
					real m_motor_max_force = motor_max_force;
					auto m_constraint1 = constraint1;
					auto m_constraint2 = constraint2;
			
					while (true) {
						std::this_thread::sleep_for(std::chrono::milliseconds(10000));
						sign *= -1;
						m_constraint1->enableMotor(sign * m_motor_speed, m_motor_max_force);
						m_constraint2->enableMotor(sign * m_motor_speed, m_motor_max_force);
					}
				});

				flipMotorThread.detach();
			}

			if (event.idata1 == GLFW_KEY_F && event.idata3 == GLFW_PRESS) {
				glmvec3 positionCamera{ getSceneManagerPointer()->getSceneNode("StandardCameraParent")->getWorldTransform()[3] };
				glmvec3 dir{ getSceneManagerPointer()->getSceneNode("StandardCamera")->getWorldTransform()[2] };
				positionCamera[1] += 3.0_real;

				glmvec3 cubePos1 = positionCamera + 2.0_real * dir;
				glmvec3 cubePos2 = cubePos1;
				cubePos2[0] += 2.0;
				cubePos2[1] += 2.0;
				glmvec3 jointAnchor = 0.5_real * (cubePos1 + cubePos2);
				//glmvec3 jointAnchor = cubePos1;

				VESceneNode* cube0;
				VECHECKPOINTER(cube0 = getSceneManagerPointer()->loadModel("The Cube" + std::to_string(m_physics->m_body_id), "media/models/test/crate0", "cube.obj", 0, getRoot()));
				auto body = std::make_shared<VPEWorld::Body>(m_physics, "Body" + std::to_string(m_physics->m_bodies.size()), cube0, &m_physics->g_cube, glmvec3{ 1.0_real }, cubePos1, glmquat{ 1,0,0,0 }, glmvec3{ 0.0_real }, glmvec3{ 0.0_real }, 1.0_real / 100.0_real, m_physics->m_restitution, m_physics->m_friction);
					body->setForce(0ul, VPEWorld::Force{ {0, m_physics->c_gravity, 0} });
				body->m_on_move = onMove;
				body->m_on_erase = onErase;
				m_physics->addBody(body);

				VESceneNode* cube1;
				VECHECKPOINTER(cube1 = getSceneManagerPointer()->loadModel("The Cube" + std::to_string(m_physics->m_body_id), "media/models/test/crate0", "cube.obj", 0, getRoot()));
				auto body1 = std::make_shared<VPEWorld::Body>(m_physics, "Body" + std::to_string(m_physics->m_bodies.size()), cube1, &m_physics->g_cube, glmvec3{ 1.0_real }, cubePos2, glmquat{ 1,0,0,0 }, glmvec3{ 0.0_real }, glmvec3{ 0.0_real }, 1.0_real / 100.0_real, m_physics->m_restitution, m_physics->m_friction);
				body1->m_on_move = onMove;
				body1->m_on_erase = onErase;
				body1->setForce(0ul, VPEWorld::Force{ {0, m_physics->c_gravity, 0} });
				m_physics->addBody(body1);

				auto constraint = std::make_shared<VPEWorld::FixedJoint>(body, body1, jointAnchor);
				m_physics->addConstraint(constraint);
			}

			if (event.idata1 == GLFW_KEY_U && event.idata3 == GLFW_PRESS) {
				glmvec3 positionCamera{ getSceneManagerPointer()->getSceneNode("StandardCameraParent")->getWorldTransform()[3] };
				glmvec3 dir{ getSceneManagerPointer()->getSceneNode("StandardCamera")->getWorldTransform()[2] };
				positionCamera[1] += 3.0_real;

				glmvec3 cubePos1 = positionCamera + 2.0_real * dir;
				glmvec3 cubePos2 = cubePos1;
				cubePos2[2] += 6.0;
				//glmvec3 jointAnchor = 0.5_real * (cubePos1 + cubePos2);
				glmvec3 jointAnchor = cubePos1;

				VESceneNode* cube0;
				VECHECKPOINTER(cube0 = getSceneManagerPointer()->loadModel("The Cube" + std::to_string(m_physics->m_body_id), "media/models/test/crate0", "cube.obj", 0, getRoot()));
				auto body = std::make_shared<VPEWorld::Body>(m_physics, "Body" + std::to_string(m_physics->m_bodies.size()), cube0, &m_physics->g_cube, glmvec3{ 1.0_real }, cubePos1, glmquat{ 1,0,0,0 }, glmvec3{ 0.0_real }, glmvec3{ 0.0_real }, 0.0_real, m_physics->m_restitution, m_physics->m_friction);
			//	body->setForce(0ul, VPEWorld::Force{ {0, m_physics->c_gravity, 0} });
				body->m_on_move = onMove;
				body->m_on_erase = onErase;
				m_physics->addBody(body);

				VESceneNode* cube1;
				VECHECKPOINTER(cube1 = getSceneManagerPointer()->loadModel("The Cube" + std::to_string(m_physics->m_body_id), "media/models/test/crate0", "cube.obj", 0, getRoot()));
				auto body1 = std::make_shared<VPEWorld::Body>(m_physics, "Body" + std::to_string(m_physics->m_bodies.size()), cube1, &m_physics->g_cube, glmvec3{ 1.0_real }, cubePos2, glmquat{ 1,0,0,0 }, glmvec3{ 0.0_real }, glmvec3{ 0.0_real }, 1.0_real / 100.0_real, m_physics->m_restitution, m_physics->m_friction);
				body1->m_on_move = onMove;
				body1->m_on_erase = onErase;
				body1->setForce(0ul, VPEWorld::Force{ {0, m_physics->c_gravity, 0} });
				m_physics->addBody(body1);

				glmvec3 jointAxis{ 0.0_real, 0.0_real, 1.0_real };
				auto constraint = std::make_shared<VPEWorld::SliderJoint>(body, body1, jointAnchor, jointAxis);
				m_physics->addConstraint(constraint);
				//constraint->enableLimit(0, 3);
			}

			return false;
		};

		VPEWorld* m_physics;	//Pointer to the physics world

	public:
		///Constructor of class EventListenerCollision
		VEEventListenerPhysicsKeys(std::string name, VPEWorld* physics) : VEEventListener(name), m_physics{physics} { };

		///Destructor of class EventListenerCollision
		virtual ~VEEventListenerPhysicsKeys() {};
	};


	//---------------------------------------------------------------------------------------------------------
	//Listener for creating the debug GUI

	class VEEventListenerPhysicsGUI : public VEEventListener
	{

	protected:
		std::default_random_engine rnd_gen{ 12345 };					//Random numbers
		std::uniform_real_distribution<> rnd_unif{ 0.0f, 1.0f };		//Random numbers

		virtual void onDrawOverlay(veEvent event) {
			//return;
			VESubrender_Nuklear* pSubrender = (VESubrender_Nuklear*)getEnginePointer()->getRenderer()->getOverlay();
			if (pSubrender == nullptr)
				return;

			struct nk_context* ctx = pSubrender->getContext();

			/* GUI */
			if (nk_begin(ctx, "Physics Panel", nk_rect(20, 20, 550, 900),
				NK_WINDOW_BORDER | NK_WINDOW_MOVABLE | NK_WINDOW_SCALABLE |
				NK_WINDOW_MINIMIZABLE | NK_WINDOW_TITLE))
			{
				std::stringstream str;
				str << std::setprecision(5);

				nk_layout_row_dynamic(ctx, 60, 2);
				if (nk_option_label(ctx, "Solver A", m_physics->m_solver == 0)) m_physics->m_solver = 0;
				if (nk_option_label(ctx, "Solver B", m_physics->m_solver == 1)) m_physics->m_solver = 1;

				str << "Sim Freq " << m_physics->m_sim_frequency;
				nk_layout_row_dynamic(ctx, 30, 4);
				nk_label(ctx, str.str().c_str(), NK_TEXT_LEFT);
				if (nk_button_label(ctx, "-10")) {
					m_physics->m_sim_frequency = std::max(10.0_real, (real)m_physics->m_sim_frequency - 10.0_real);
					m_physics->m_sim_delta_time = 1.0_real / m_physics->m_sim_frequency;
				}
				if (nk_button_label(ctx, "+10")) {
					m_physics->m_sim_frequency += 10;
					m_physics->m_sim_delta_time = 1.0_real / m_physics->m_sim_frequency;
				}
				if (nk_button_label(ctx, "Next time slot")) {
					m_physics->m_current_time += m_physics->m_sim_delta_time;
				}

				nk_layout_row_dynamic(ctx, 30, 2);
				if (nk_option_label(ctx, "Realtime", m_physics->m_mode == VPEWorld::simulation_mode_t::SIMULATION_MODE_REALTIME))
					m_physics->m_mode = VPEWorld::simulation_mode_t::SIMULATION_MODE_REALTIME;
				if (nk_option_label(ctx, "Debug", m_physics->m_mode == VPEWorld::simulation_mode_t::SIMULATION_MODE_DEBUG))
					m_physics->m_mode = VPEWorld::simulation_mode_t::SIMULATION_MODE_DEBUG;

				nk_layout_row_begin(ctx, NK_STATIC, 30, 2);
				nk_layout_row_push(ctx, 60);
				nk_label(ctx, "Time (s)", NK_TEXT_LEFT);
				str.str("");
				str << m_physics->m_current_time;
				nk_layout_row_push(ctx, 60);
				nk_label(ctx, str.str().c_str(), NK_TEXT_LEFT);
				nk_layout_row_end(ctx);

				static real fps = 0.0;
				fps = 0.05_real * m_physics->m_fps + 0.95_real * fps;
				str.str("");
				str << "FPS " << fps;
				nk_layout_row_dynamic(ctx, 30, 1);
				nk_label(ctx, str.str().c_str(), NK_TEXT_LEFT);

				str.str("");
				str << "Loops " << m_physics->m_loops;
				nk_layout_row_dynamic(ctx, 30, 3);
				nk_label(ctx, str.str().c_str(), NK_TEXT_LEFT);
				if (nk_button_label(ctx, "-5")) { m_physics->m_loops = std::max(5, m_physics->m_loops - 5); }
				if (nk_button_label(ctx, "+5")) { m_physics->m_loops += 5; }

				str.str("");
				str << "Resting Fac " << m_physics->m_resting_factor;
				nk_layout_row_dynamic(ctx, 30, 3);
				nk_label(ctx, str.str().c_str(), NK_TEXT_LEFT);
				if (nk_button_label(ctx, "-0.2")) { m_physics->m_resting_factor = std::max(0.2_real, m_physics->m_resting_factor - 0.2_real); }
				if (nk_button_label(ctx, "+0.2")) { m_physics->m_resting_factor += 0.2_real; }

				str.str("");
				str << "Damp Incr " << m_physics->m_damping_incr;
				nk_layout_row_dynamic(ctx, 30, 3);
				nk_label(ctx, str.str().c_str(), NK_TEXT_LEFT);
				if (nk_button_label(ctx, "-5")) { m_physics->m_damping_incr = std::max(0.0_real, m_physics->m_damping_incr - 5.0_real); }
				if (nk_button_label(ctx, "+5")) { m_physics->m_damping_incr += 5.0_real; }

				str.str("");
				str << "PBias Fac " << m_physics->m_pbias_factor;
				nk_layout_row_dynamic(ctx, 30, 3);
				nk_label(ctx, str.str().c_str(), NK_TEXT_LEFT);
				if (nk_button_label(ctx, "-0.1")) { m_physics->m_pbias_factor = glm::clamp(m_physics->m_pbias_factor - 0.1_real, 0.0_real, 1.0_real); }
				if (nk_button_label(ctx, "+0.1")) { m_physics->m_pbias_factor = glm::clamp(m_physics->m_pbias_factor + 0.1_real, 0.0_real, 1.0_real); }

				nk_layout_row_dynamic(ctx, 30, 3);
				nk_label(ctx, "Align PBias", NK_TEXT_LEFT);
				if (nk_option_label(ctx, "Yes", m_physics->m_align_position_bias == 1))
					m_physics->m_align_position_bias = 1;
				if (nk_option_label(ctx, "No", m_physics->m_align_position_bias == 0))
					m_physics->m_align_position_bias = 0;

				nk_layout_row_dynamic(ctx, 30, 3);
				nk_label(ctx, "Use VBias", NK_TEXT_LEFT);
				if (nk_option_label(ctx, "Yes", m_physics->m_use_vbias == 1))
					m_physics->m_use_vbias = 1;
				if (nk_option_label(ctx, "No", m_physics->m_use_vbias == 0))
					m_physics->m_use_vbias = 0;

				nk_layout_row_dynamic(ctx, 30, 3);
				nk_label(ctx, "Warmstart All", NK_TEXT_LEFT);
				if (nk_option_label(ctx, "Yes", m_physics->m_use_warmstart == 1))
					m_physics->m_use_warmstart = 1;
				if (nk_option_label(ctx, "No", m_physics->m_use_warmstart == 0))
					m_physics->m_use_warmstart = 0;

				nk_layout_row_dynamic(ctx, 30, 3);
				nk_label(ctx, "Warmstart Single", NK_TEXT_LEFT);
				if (nk_option_label(ctx, "Yes", m_physics->m_use_warmstart_single == 1))
					m_physics->m_use_warmstart_single = 1;
				if (nk_option_label(ctx, "No", m_physics->m_use_warmstart_single == 0))
					m_physics->m_use_warmstart_single = 0;

				nk_layout_row_dynamic(ctx, 30, 3);
				nk_label(ctx, "Deactivate", NK_TEXT_LEFT);
				if (nk_option_label(ctx, "Yes", m_physics->m_deactivate))
					m_physics->m_deactivate = true;
				if (nk_option_label(ctx, "No", !m_physics->m_deactivate))
					m_physics->m_deactivate = false;

				nk_layout_row_dynamic(ctx, 30, 3);
				nk_label(ctx, "Clamp Pos", NK_TEXT_LEFT);
				if (nk_option_label(ctx, "Yes", m_physics->m_clamp_position == 1))
					m_physics->m_clamp_position = 1;
				if (nk_option_label(ctx, "No", m_physics->m_clamp_position == 0))
					m_physics->m_clamp_position = 0;

				str.str("");
				str << "Num Bodies " << m_physics->m_bodies.size();
				nk_layout_row_dynamic(ctx, 30, 3);
				nk_label(ctx, str.str().c_str(), NK_TEXT_LEFT);
				if (nk_button_label(ctx, "Create Bodies")) {
					createRandomBodies(20);
				}
				if (nk_button_label(ctx, "Clear Bodies")) {
					m_physics->clear();
				}

				str.str("");
				str << "Num Contacts " << m_physics->m_contacts.size();
				nk_layout_row_dynamic(ctx, 30, 1);
				nk_label(ctx, str.str().c_str(), NK_TEXT_LEFT);

				str.str("");
				str << "Num Active " << m_physics->m_num_active;
				nk_layout_row_dynamic(ctx, 30, 1);
				nk_label(ctx, str.str().c_str(), NK_TEXT_LEFT);

				str.str("");
				str << "Cell width " << m_physics->m_width;
				nk_layout_row_dynamic(ctx, 30, 3);
				nk_label(ctx, str.str().c_str(), NK_TEXT_LEFT);
				if (nk_button_label(ctx, "-1")) {
					m_physics->m_grid.clear();
					m_physics->m_width = std::max(1.0_real, m_physics->m_width - 1);
					for (auto body : m_physics->m_bodies) m_physics->addGrid(body.second);
				}
				if (nk_button_label(ctx, "+1")) {
					m_physics->m_grid.clear();
					m_physics->m_width++;
					for (auto body : m_physics->m_bodies) m_physics->addGrid(body.second);
				}

				nk_layout_row_dynamic(ctx, 30, 5);
				str.str("Current Body ");
				if (m_physics->m_body) { str << "Current Body " << m_physics->m_body->m_name; }
				nk_label(ctx, str.str().c_str(), NK_TEXT_LEFT);

				if (nk_button_label(ctx, "Pick body")) { 
					glmvec3 pos{ getSceneManagerPointer()->getSceneNode("StandardCameraParent")->getWorldTransform()[3] };
					glmvec3 dir{ getSceneManagerPointer()->getSceneNode("StandardCamera")->getWorldTransform()[2] };
					m_physics->m_body = m_physics->pickBody( pos, dir );
				}
				if (nk_button_label(ctx, "Delete body")) {
					glmvec3 pos{ getSceneManagerPointer()->getSceneNode("StandardCameraParent")->getWorldTransform()[3] };
					glmvec3 dir{ getSceneManagerPointer()->getSceneNode("StandardCamera")->getWorldTransform()[2] };
					auto b = m_physics->pickBody( pos, dir );
					if(b) m_physics->eraseBody(b);
				}
				if (nk_button_label(ctx, "Add collider")) {
					if(m_physics->m_body)
						m_physics->addCollider(m_physics->m_body, onCollide);
				}
				if (nk_button_label(ctx, "Remove colliders")) {
					m_physics->clearCollider();
				}

				real vel = 5.0;
				real m_dx, m_dy, m_dz, m_da, m_db, m_dc;
				m_dx = m_dy = m_dz = m_da = m_db = m_dc = 0.0;

				nk_layout_row_static(ctx, 30, 100, 2);
				if (nk_button_label(ctx, "+X")) {m_dx = vel; }
				if (nk_button_label(ctx, "-X")) { m_dx = -vel; }
				nk_layout_row_static(ctx, 30, 100, 2);
				if (nk_button_label(ctx, "+Y")) { m_dy = vel; }
				if (nk_button_label(ctx, "-Y")) { m_dy = -vel; }
				nk_layout_row_static(ctx, 30, 100, 2);
				if (nk_button_label(ctx, "+Z")) { m_dz = vel; }
				if (nk_button_label(ctx, "-Z")) {m_dz = -vel; }

				nk_layout_row_static(ctx, 30, 100, 2);
				if (nk_button_label(ctx, "RX")) { m_da = vel; }
				if (nk_button_label(ctx, "-RX")) { m_da = -vel; }
				nk_layout_row_static(ctx, 30, 100, 2);
				if (nk_button_label(ctx, "+RY")) { m_db = vel; }
				if (nk_button_label(ctx, "-RY")) { m_db = -vel; }
				nk_layout_row_static(ctx, 30, 100, 2);
				if (nk_button_label(ctx, "+RZ")) { m_dc = vel; }
				if (nk_button_label(ctx, "-RZ")) { m_dc = -vel; }

				if (m_physics->m_body) {
					real dt = (real)m_physics->m_sim_delta_time;
					m_physics->m_body->m_positionW += (real)dt * glmvec3{ m_dx, m_dy, m_dz };
					m_physics->m_body->m_orientationLW =
						glm::rotate(glmquat{ 1,0,0,0 }, (real)dt * m_da, glmvec3{ 1, 0, 0 }) *
						glm::rotate(glmquat{ 1,0,0,0 }, (real)dt * m_db, glmvec3{ 0, 1, 0 }) *
						glm::rotate(glmquat{ 1,0,0,0 }, (real)dt * m_dc, glmvec3{ 0, 0, 1 }) * m_physics->m_body->m_orientationLW;

					m_physics->m_body->updateMatrices();
					m_dx = m_dy = m_dz = m_da = m_db = m_dc = 0.0_real;
				}
			}
			nk_end(ctx);
		}

		/// <summary>
		/// Create a number of bodies at random places to populate the scene.
		/// </summary>
		/// <param name="n">Number of bodies to create.</param>
		void createRandomBodies(int n) {
			for (int i = 0; i < n; ++i) {
				glmvec3 pos = { rnd_unif(rnd_gen), 20 * rnd_unif(rnd_gen) + 10.0_real, rnd_unif(rnd_gen) };
				glmvec3 vel = { rnd_unif(rnd_gen), rnd_unif(rnd_gen), rnd_unif(rnd_gen) };
				glmvec3 scale{ 1,1,1 }; // = rnd_unif(rnd_gen) * 10;
				real angle = (real)rnd_unif(rnd_gen) * 10 * 3 * (real)M_PI / 180.0_real;
				glmvec3 orient{ rnd_unif(rnd_gen), rnd_unif(rnd_gen), rnd_unif(rnd_gen) };
				glmvec3 vrot{ rnd_unif(rnd_gen) * 5, rnd_unif(rnd_gen) * 5, rnd_unif(rnd_gen) * 5 };
				VESceneNode* cube;
				VECHECKPOINTER(cube = getSceneManagerPointer()->loadModel("The Cube" + std::to_string(m_physics->m_body_id), "media/models/test/crate0", "cube.obj", 0, getRoot()));
				auto body = std::make_shared<VPEWorld::Body>( m_physics, "Body" + std::to_string(m_physics->m_bodies.size()), cube, &VPEWorld::g_cube, scale, pos, glm::rotate(angle, glm::normalize(orient)), vel, vrot, 1.0_real / 100.0_real, m_physics->m_restitution, m_physics->m_friction );
				body->m_forces.insert({ 0ul, VPEWorld::Force{ {0, m_physics->c_gravity, 0} } });
				body->m_on_move = onMove;
				body->m_on_erase = onErase;
				m_physics->addBody(m_physics->m_body = body);
			}
		}


		VPEWorld* m_physics;	//pointer to the physics world

	public:

		///Constructor of class VEEventListenerPhysicsGUI
		VEEventListenerPhysicsGUI(std::string name, VPEWorld* phy) : VEEventListener{ name }, m_physics{ phy } {};

		///Destructor of class VEEventListenerPhysicsGUI
		virtual ~VEEventListenerPhysicsGUI() {};
	};


	//---------------------------------------------------------------------------------------------------------
	//My custom engine

	///user defined manager class, derived from VEEngine
	class MyVulkanEngine : public VEEngine {
	public:

		VPEWorld					m_physics;
		VEEventListenerPhysics*		m_physics_listener;
		VEEventListenerPhysicsKeys* m_physics_listener_keys;
		VEEventListenerPhysicsGUI*	m_physics_listener_gui;

		MyVulkanEngine(veRendererType type = veRendererType::VE_RENDERER_TYPE_FORWARD, bool debug=false) : VEEngine(type, debug) {};
		~MyVulkanEngine() {};

		///Register an event listener to interact with the user
		
		virtual void registerEventListeners() {
			VEEngine::registerEventListeners();

			registerEventListener(m_physics_listener = new VEEventListenerPhysics("Physics", &m_physics), { veEvent::VE_EVENT_FRAME_STARTED });
			registerEventListener(m_physics_listener_keys = new VEEventListenerPhysicsKeys("Physics Keys", &m_physics), { veEvent::VE_EVENT_KEYBOARD });
			registerEventListener(m_physics_listener_gui = new VEEventListenerPhysicsGUI("Physics GUI",&m_physics), { veEvent::VE_EVENT_DRAW_OVERLAY });
		};
		

		///Load the first level into the game engine
		///The engine uses Y-UP, Left-handed
		virtual void loadLevel( uint32_t numLevel=1) {

			VEEngine::loadLevel(numLevel );			//create standard cameras and lights

			VESceneNode *pScene;
			VECHECKPOINTER( pScene = getSceneManagerPointer()->createSceneNode("Level 1", getRoot()) );
	
			//scene models

			VESceneNode *sp1;
			VECHECKPOINTER( sp1 = getSceneManagerPointer()->createSkybox("The Sky", "media/models/test/sky/cloudy",
										{	"bluecloud_ft.jpg", "bluecloud_bk.jpg", "bluecloud_up.jpg", 
											"bluecloud_dn.jpg", "bluecloud_rt.jpg", "bluecloud_lf.jpg" }, pScene)  );

			VESceneNode *e4;
			VECHECKPOINTER( e4 = getSceneManagerPointer()->loadModel("The Plane", "media/models/test/plane", "plane_t_n_s.obj",0, pScene) );
			e4->setTransform(glm::scale( glm::translate( glm::vec3{ 0,0,0,}) , glm::vec3(1000.0f, 1.0f, 1000.0f)));

			VEEntity *pE4;
			VECHECKPOINTER( pE4 = (VEEntity*)getSceneManagerPointer()->getSceneNode("The Plane/plane_t_n_s.obj/plane/Entity_0") );
			pE4->setParam( glm::vec4(1000.0f, 1000.0f, 0.0f, 0.0f) );

			getSceneManagerPointer()->getSceneNode("StandardCameraParent")->setPosition({0,1,-4});

			//VESceneNode *e1,*eParent;
			//eParent = getSceneManagerPointer()->createSceneNode("The Cube Parent", pScene, glm::mat4(1.0));
			//VECHECKPOINTER(e1 = getSceneManagerPointer()->loadModel("The Cube0", "media/models/test/crate0", "cube.obj"));
			//eParent->multiplyTransform(glm::translate(glm::mat4(1.0f), glm::vec3(-10.0f, 1.0f, 10.0f)));
			//eParent->addChild(e1);
		};
	};
}


//-------------------------------------------------------------------------------------------------------

using namespace ve;

int main() {
	bool debug = false;

	MyVulkanEngine mve(veRendererType::VE_RENDERER_TYPE_FORWARD, debug);	//enable or disable debugging (=callback, validation layers)
	mve.initEngine();
	mve.loadLevel(1);
	mve.run();

	return 0;
}
