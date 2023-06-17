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

	//----------------------------------------------------------------------------------------------
	//callbacks for bodies

	/// <summary>
	/// This callback is used for updating the visual body whenever a physics body moves.
	/// It is also used for extrapolating the new position between two simulation slots.
	/// </summary>
	inline VPEWorld::callback_move onMove =
		[&](double dt, std::shared_ptr<VPEWorld::Body> body) {
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
	inline VPEWorld::callback_erase onErase =
		[&](std::shared_ptr<VPEWorld::Body> body) {
			VESceneNode* node = static_cast<VESceneNode*>(body->m_owner);							//Owner is a pointer to a scene node
			getSceneManagerPointer()->deleteSceneNodeAndChildren(
			((VESceneNode*)body->m_owner)->getName());
	};

	//--------------------------------Begin-Cloth-Simulation-Stuff----------------------------------
	// by Felix Neumann

	/// <summary>
	/// Called by the cloth of it moves.
	/// </summary>
	/// <param name="dt"> Delta time.</param>
	/// <param name="cloth"> Pointer to the cloth so that the owner can get the data. </param>
	inline VPEWorld::callback_move_cloth onMoveCloth =
		[&](double dt, std::shared_ptr<VPEWorld::Cloth> cloth)
	{
		VEClothEntity* clothOwner = static_cast<VEClothEntity*>(cloth->m_owner);					// Owner is a pointer to a scene node
		auto vertices = cloth->generateVertices();													// Vertices with updated position data
		(static_cast<VEClothMesh*> (clothOwner->m_pMesh))->updateVertices(vertices);				// Update the vertices of the mesh
	};

	/// <summary>
	/// Called by the cloth if it is erased.
	/// </summary>
	/// <param name="cloth"> Shared pointer to the cloth. </param>
	inline VPEWorld::callback_erase_cloth onEraseCloth =
		[&](std::shared_ptr<VPEWorld::Cloth> cloth) {
		VESceneNode* node = static_cast<VESceneNode*>(cloth->m_owner);								// Owner is a pointer to a scene node
		getSceneManagerPointer()->deleteSceneNodeAndChildren(										// Delete the owner and child node
			((VESceneNode*)cloth->m_owner)->getName());												// associated with the cloth
	};

	//---------------------------------End-Cloth-Simulation-Stuff-----------------------------------

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
		void onFrameStarted(veEvent event) { m_physics->tick(event.dt); }

		VPEWorld* m_physics;	//Pointer to the physics world
		VEClothEntity* p_clothEntity;

	public:
		///Constructor of class EventListenerCollision
		VEEventListenerPhysics(std::string name, VPEWorld* physics) : VEEventListener(name), m_physics{ physics } {};

		///Destructor of class EventListenerCollision
		virtual ~VEEventListenerPhysics() {};
	};

	//---------------------------------------------------------------------------------------------------------
	//Listener for creating bodies with keyboard

	/// <summary>
	/// This is a callback that is called in each loop. It implements a simple rigid body 
	/// physics engine.
	/// </summary>
	class VEEventListenerPhysicsKeys : public VEEventListener {

		std::default_random_engine rnd_gen{ 12345 };					//Random numbers
		std::uniform_real_distribution<> rnd_unif{ 0.0f, 1.0f };		//Random numbers

		static constexpr float SPEED = 3.0f;

	public:

		/// <summary>
		/// Callback for event key stroke. Depending on the key pressed, bodies are created.
		/// </summary>
		/// <param name="event">The keyboard event.</param>
		/// <returns>False, so the key is not consumed.</returns>
		bool onKeyboard(veEvent event) {
			// Cubes Controls
			{
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
					auto body = std::make_shared<VPEWorld::Body>(m_physics, "Body" + std::to_string(m_physics->m_bodies.size()), cube, &m_physics->g_cube, scale, positionCamera + 2.0_real * dir, glm::rotate(angle, glm::normalize(orient)), vel, vrot, 1.0_real / 100.0_real, m_physics->m_restitution, m_physics->m_friction);
					body->setForce(0ul, VPEWorld::Force{ {0, m_physics->c_gravity, 0} });
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
						auto body = std::make_shared<VPEWorld::Body>(m_physics, "Body" + std::to_string(m_physics->m_bodies.size()), cube0, &m_physics->g_cube, glmvec3{ 1.0_real }, glmvec3{ positionCamera.x, dy++, positionCamera.z + 4 }, glmquat{ 1,0,0,0 }, glmvec3{ 0.0_real }, glmvec3{ 0.0_real }, 1.0_real / 100.0_real, m_physics->m_restitution, m_physics->m_friction);
						body->setForce(0ul, VPEWorld::Force{ {0, m_physics->c_gravity, 0} });
						body->m_orientationLW = glm::toQuat(glm::rotate(glm::radians(10.f), glm::vec3(0, 1, 0)));
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
							auto body = std::make_shared<VPEWorld::Body>(m_physics, "Body" + std::to_string(m_physics->m_bodies.size()), cube0, &m_physics->g_cube, glmvec3{ 1.0_real }, glmvec3{ dx + 0.4 * dy, 0.5_real + dy, 0.0_real }, glmquat{ 1,0,0,0 }, glmvec3{ 0.0_real }, glmvec3{ 0.0_real }, 1.0_real / 100.0_real, m_physics->m_restitution, m_physics->m_friction);
							body->setForce(0ul, VPEWorld::Force{ {0, m_physics->c_gravity, 0} });
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
					auto body = std::make_shared<VPEWorld::Body>(m_physics, "Body" + std::to_string(m_physics->m_bodies.size()), cube0, &m_physics->g_cube, glmvec3{ 1.0_real }, positionCamera + 2.0_real * dir, glmquat{ 1,0,0,0 }, glmvec3{ 0.0_real }, glmvec3{ 0.0_real }, 1.0_real / 100.0_real, m_physics->m_restitution, m_physics->m_friction);
					body->setForce(0ul, VPEWorld::Force{ {0, m_physics->c_gravity, 0} });
					body->m_on_move = onMove;
					body->m_on_erase = onErase;
					m_physics->addBody(body);
				}
			}

			// Cloth Controls
			{
				VESceneNode* cloth = getSceneManagerPointer()->getSceneNode("Cloth");

				if (event.idata1 == GLFW_KEY_L) {
					m_physics->getCloth(cloth)->applyTransformation(glm::translate(glm::mat4(1.0f),
						glm::vec3(SPEED * event.dt, 0.0f, 0.0f)), true);
				}

				if (event.idata1 == GLFW_KEY_J) {
					m_physics->getCloth(cloth)->applyTransformation(glm::translate(glm::mat4(1.0f),
						glm::vec3(-SPEED * event.dt, 0.0f, 0.0f)), true);
				}

				if (event.idata1 == GLFW_KEY_O) {
					m_physics->getCloth(cloth)->applyTransformation(glm::translate(glm::mat4(1.0f),
						glm::vec3(0.0f, SPEED * event.dt, 0.0f)), true);
				}

				if (event.idata1 == GLFW_KEY_U) {
					m_physics->getCloth(cloth)->applyTransformation(glm::translate(glm::mat4(1.0f),
						glm::vec3(0.0f, -SPEED * event.dt, 0.0f)), true);
				}

				if (event.idata1 == GLFW_KEY_I) {
					m_physics->getCloth(cloth)->applyTransformation(glm::translate(glm::mat4(1.0f),
						glm::vec3(0.0f, 0.0f, SPEED * event.dt)), true);
				}

				if (event.idata1 == GLFW_KEY_K) {
					m_physics->getCloth(cloth)->applyTransformation(glm::translate(glm::mat4(1.0f),
						glm::vec3(0.0f, 0.0f, -SPEED * event.dt)), false);
				}
			}

			return false;
		};

		VPEWorld* m_physics;	//Pointer to the physics world

	public:
		///Constructor of class EventListenerCollision
		VEEventListenerPhysicsKeys(std::string name, VPEWorld* physics) : VEEventListener(name),
			m_physics{physics} { };

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
		VPEWorld* m_physics;	//pointer to the physics world

		virtual void onDrawOverlay(veEvent event) {
			VESubrender_Nuklear* pSubrender =
				(VESubrender_Nuklear*)getEnginePointer()->getRenderer()->getOverlay();

			if (pSubrender == nullptr)
				return;

			struct nk_context* ctx = pSubrender->getContext();

			/* GUI */
			if (nk_begin(ctx, "Cloth Sim Demo", nk_rect(20, 20, 330, 900),
				NK_WINDOW_BORDER | NK_WINDOW_MOVABLE | NK_WINDOW_SCALABLE |
				NK_WINDOW_MINIMIZABLE | NK_WINDOW_TITLE))
			{
				std::stringstream str;
				str << std::setprecision(5);

				/*
				str << "Sim Freq " << m_physics->m_sim_frequency;
				nk_layout_row_dynamic(ctx, 30, 1);
				nk_label(ctx, str.str().c_str(), NK_TEXT_LEFT);

				nk_layout_row_static(ctx, NK_STATIC, 30, 3);
				if (nk_button_label(ctx, "-10")) {
					m_physics->m_sim_frequency = std::max(10.0_real, (real)m_physics->m_sim_frequency - 10.0_real);
					m_physics->m_sim_delta_time = 1.0_real / m_physics->m_sim_frequency;
				}
				if (nk_button_label(ctx, "+10")) {
					m_physics->m_sim_frequency += 10;
					m_physics->m_sim_delta_time = 1.0_real / m_physics->m_sim_frequency;
				}
				*/

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
				nk_layout_row_dynamic(ctx, 30, 1);
				nk_label(ctx, str.str().c_str(), NK_TEXT_LEFT);
				nk_layout_row_static(ctx, 30, 150, 2);
				if (nk_button_label(ctx, "-5"))
					m_physics->m_loops = std::max(5, m_physics->m_loops - 5);
				if (nk_button_label(ctx, "+5"))
					m_physics->m_loops += 5;

				nk_layout_row_dynamic(ctx, 30, 1);
				nk_label(ctx, "", NK_TEXT_LEFT);

				nk_layout_row_dynamic(ctx, 30, 1);
				nk_label(ctx, "Rigid Body Controls", NK_TEXT_LEFT);
				nk_layout_row_static(ctx, 30, 150, 2);
				if (nk_button_label(ctx, "Create Cube"))
					createRandomBodies(20);
				if (nk_button_label(ctx, "Clear Cubes"))
					m_physics->clear();

				nk_layout_row_dynamic(ctx, 30, 1);
				str.str("Current Cube: ");
				if (m_physics->m_body)
					str << "Current Cube: " << m_physics->m_body->m_name;
				nk_label(ctx, str.str().c_str(), NK_TEXT_LEFT);

				nk_layout_row_static(ctx, 30, 150, 2);
				if (nk_button_label(ctx, "Pick Cube")) { 
					glmvec3 pos{ getSceneManagerPointer()->getSceneNode("StandardCameraParent")->getWorldTransform()[3] };
					glmvec3 dir{ getSceneManagerPointer()->getSceneNode("StandardCamera")->getWorldTransform()[2] };
					m_physics->m_body = m_physics->pickBody( pos, dir );
				}
				if (nk_button_label(ctx, "Delete Cube")) {
					glmvec3 pos{ getSceneManagerPointer()->getSceneNode("StandardCameraParent")->getWorldTransform()[3] };
					glmvec3 dir{ getSceneManagerPointer()->getSceneNode("StandardCamera")->getWorldTransform()[2] };
					auto b = m_physics->pickBody( pos, dir );
					if(b) m_physics->eraseBody(b);
				}

				real vel = 5.0;
				real m_dx, m_dy, m_dz, m_da, m_db, m_dc;
				m_dx = m_dy = m_dz = m_da = m_db = m_dc = 0.0;

				nk_layout_row_static(ctx, 30, 150, 2);
				if (nk_button_label(ctx, "+X")) { m_dx = vel; }
				if (nk_button_label(ctx, "-X")) { m_dx = -vel; }
				nk_layout_row_static(ctx, 30, 150, 2);
				if (nk_button_label(ctx, "+Y")) { m_dy = vel; }
				if (nk_button_label(ctx, "-Y")) { m_dy = -vel; }
				nk_layout_row_static(ctx, 30, 150, 2);
				if (nk_button_label(ctx, "+Z")) { m_dz = vel; }
				if (nk_button_label(ctx, "-Z")) { m_dz = -vel; }

				nk_layout_row_static(ctx, 30, 150, 2);
				if (nk_button_label(ctx, "+RX")) { m_da = vel; }
				if (nk_button_label(ctx, "-RX")) { m_da = -vel; }
				nk_layout_row_static(ctx, 30, 150, 2);
				if (nk_button_label(ctx, "+RY")) { m_db = vel; }
				if (nk_button_label(ctx, "-RY")) { m_db = -vel; }
				nk_layout_row_static(ctx, 30, 150, 2);
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

				nk_layout_row_dynamic(ctx, 30, 1);
				nk_label(ctx, "Create Cube: 'SPACE'", NK_TEXT_LEFT);
				nk_layout_row_dynamic(ctx, 30, 1);
				nk_label(ctx, "Create Cube Pyramid: 'Z'", NK_TEXT_LEFT);
				nk_layout_row_dynamic(ctx, 30, 1);
				nk_label(ctx, "Shoot Cube: 'B'", NK_TEXT_LEFT);

				nk_layout_row_dynamic(ctx, 30, 1);
				nk_label(ctx, "", NK_TEXT_LEFT);

				nk_layout_row_dynamic(ctx, 30, 1);
				nk_label(ctx, "Cloth Body Controls", NK_TEXT_LEFT);

				nk_layout_row_dynamic(ctx, 30, 1);
				nk_label(ctx, "Move along x : 'J' 'L'", NK_TEXT_LEFT);
				nk_layout_row_dynamic(ctx, 30, 1);
				nk_label(ctx, "Move along z: 'I' 'K'", NK_TEXT_LEFT);
				nk_layout_row_dynamic(ctx, 30, 1);
				nk_label(ctx, "Move along y: 'O' 'U'", NK_TEXT_LEFT);
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

	public:
		VEEventListenerPhysicsGUI(std::string name, VPEWorld* phy) : VEEventListener{ name },
			m_physics{ phy } {};

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

		MyVulkanEngine(veRendererType type = veRendererType::VE_RENDERER_TYPE_FORWARD,
			bool debug = false) : VEEngine(type, debug) {};
		~MyVulkanEngine() {};

		///Register an event listener to interact with the user
		
		virtual void registerEventListeners() {
			VEEngine::registerEventListeners();

			registerEventListener(m_physics_listener =
				new VEEventListenerPhysics("Physics", &m_physics),
				{ veEvent::VE_EVENT_FRAME_STARTED });

			registerEventListener(m_physics_listener_keys =
				new VEEventListenerPhysicsKeys("Physics Keys", &m_physics),
				{ veEvent::VE_EVENT_KEYBOARD });

			registerEventListener(m_physics_listener_gui = 
				new VEEventListenerPhysicsGUI("Physics GUI",&m_physics),
				{ veEvent::VE_EVENT_DRAW_OVERLAY });
		};

		///Load the first level into the game engine
		///The engine uses Y-UP, Left-handed
		virtual void loadLevel(uint32_t numLevel = 1) {
			VEEngine::loadLevel(numLevel);

			getSceneManagerPointer()->getSceneNode("StandardCameraParent")->setPosition({ 0,1,-4 });

			VESceneNode *pScene;
			VECHECKPOINTER(pScene = getSceneManagerPointer()->createSceneNode("Level 1", getRoot()));
			
			//Sky and Ground Plane
			{
				VESceneNode* sp1;
				VECHECKPOINTER(sp1 = getSceneManagerPointer()->createSkybox(
					"The Sky", "media/models/test/sky/cloudy", { "bluecloud_ft.jpg",
					"bluecloud_bk.jpg", "bluecloud_up.jpg", "bluecloud_dn.jpg", "bluecloud_rt.jpg",
					"bluecloud_lf.jpg" }, pScene));

				VESceneNode* e4;
				VECHECKPOINTER(e4 = getSceneManagerPointer()->loadModel(
					"The Plane", "media/models/test/plane", "plane_t_n_s.obj", 0, pScene));
				e4->setTransform(glm::scale(glm::translate(glm::vec3{ 0,0,0, }),
					glm::vec3(1000.0f, 1.0f, 1000.0f)));

				VEEntity* pE4;
				VECHECKPOINTER(pE4 = (VEEntity*)getSceneManagerPointer()->getSceneNode(
					"The Plane/plane_t_n_s.obj/plane/Entity_0"));
				pE4->setParam(glm::vec4(1000.0f, 1000.0f, 0.0f, 0.0f));
			}

			// Cloth Cloth
			{
				VEClothEntity* clothEntity;

				VECHECKPOINTER(clothEntity = getSceneManagerPointer()->loadClothModel(
					"Cloth", "media/models/cloths/cloth1", "cloth.obj"));

				pScene->addChild(clothEntity);

				auto vertices = ((VEClothMesh*) (clothEntity->m_pMesh))->getInitialVertices();
				auto indices = ((VEClothMesh*) (clothEntity->m_pMesh))->getIndices();
				std::vector<glm::vec3> fixedPoints =
					{ {-1.000000, 2.000000, -0.000000}, {1.000000, 2.000000, 0.000000} };

				auto physicsCloth = std::make_shared<VPEWorld::Cloth>(&m_physics,
					"Cloth" + std::to_string(m_physics.m_bodies.size()), clothEntity, onMoveCloth,
					onEraseCloth, vertices, indices, fixedPoints, 50, 4, 0.8);

				m_physics.addCloth(physicsCloth);

				std::default_random_engine rnd_gen{ 12345 };					//Random numbers
				std::uniform_real_distribution<> rnd_unif{ 0.0f, 1.0f };		//Random numbers

				//std::cout << m_physics.alphaMaxPlusBetaMin(0.2, 3) << std::endl;

				//std::cout << std::sqrt(std::pow(0.2, 2) + std::pow(3, 2)) << std::endl;

				/*
				VESceneNode* cubeParent = getSceneManagerPointer()->createSceneNode("Cube Parent",
					pScene, glm::mat4(1.0));

				glmvec3 positionCamera{ getSceneManagerPointer()->getSceneNode("StandardCameraParent")->getWorldTransform()[3] };
				glmvec3 dir{ getSceneManagerPointer()->getSceneNode("StandardCamera")->getWorldTransform()[2] };
				glmvec3 vel = (30.0_real + 5.0_real * (real)rnd_unif(rnd_gen)) * dir / glm::length(dir);
				glmvec3 scale{ 1,1,1 }; // = rnd_unif(rnd_gen) * 10;
				real angle = (real)rnd_unif(rnd_gen) * 10 * 3 * (real)M_PI / 180.0_real;
				glmvec3 orient{ rnd_unif(rnd_gen), rnd_unif(rnd_gen), rnd_unif(rnd_gen) };
				glmvec3 vrot{ rnd_unif(rnd_gen) * 5, rnd_unif(rnd_gen) * 5, rnd_unif(rnd_gen) * 5 };
				VESceneNode* cube;
				VECHECKPOINTER(cube = getSceneManagerPointer()->loadModel("The Cube" + std::to_string(m_physics.m_body_id), "media/models/test/crate0", "cube.obj", 0, cubeParent));
				auto body = std::make_shared<VPEWorld::Body>(&m_physics, "Body" + std::to_string(m_physics.m_bodies.size()), cube, &m_physics.g_cube, scale, positionCamera + 2.0_real * dir, glm::rotate(angle, glm::normalize(orient)), vel, vrot, 1.0_real / 100.0_real, m_physics.m_restitution, m_physics.m_friction);
				body->setForce(0ul, VPEWorld::Force{ {0, m_physics.c_gravity, 0} });
				body->m_on_move = onMove;
				body->m_on_erase = onErase;
				m_physics.addBody(body);
				*/
			}
		};
	};
}


//-------------------------------------------------------------------------------------------------------

using namespace ve;

int main() {
	bool debug = true;

	MyVulkanEngine mve(veRendererType::VE_RENDERER_TYPE_FORWARD, debug);
	mve.initEngine();
	mve.loadLevel(1);
	mve.run();

	return 0;
}