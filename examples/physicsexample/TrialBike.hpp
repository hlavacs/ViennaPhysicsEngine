#pragma once

#include "VEInclude.h"
#include "VPE.hpp"

using namespace vpe;

namespace ve {
	/// <summary>
	/// Class that implements a few demos for constraints
	/// I put all the code in here to not clutter main.cpp and the keybinds too much
	/// </summary>
	class TrialBike {
		VPEWorld* m_physics;
		VPEWorld::callback_move m_onMove;
		VPEWorld::callback_erase m_onErase;
		std::shared_ptr<VPEWorld::HingeJoint> m_rearHinge = nullptr;
		real m_targetOmega = 6.0_real;    
		real m_maxMotorTorque = 80.0_real;   
		
	public:
		std::shared_ptr<VPEWorld::Body> m_frame;
		bool m_jumpActive = false;
		real m_jumpTimeRemaining = 0.5_real; 
		static constexpr uint64_t JUMP = 9999;
		TrialBike(VPEWorld* physics, VPEWorld::callback_move onMove, VPEWorld::callback_erase onErase) : m_physics{ physics }, m_onMove{ onMove }, m_onErase{ onErase } {}
		~TrialBike() {}

		/// <summary>
		/// Creates a cube object with the given paramteres, adds it to physics world and returns the pointer to the VPEWorld::Body created
		/// </summary>
		/// <param name="scale"></param>
		/// <param name="position"></param>
		/// <param name="orientation"></param>
		/// <param name="inv_mass"></param>
		/// <param name="gravity"></param>
		/// <param name="friction"></param>
		/// <returns></returns>
		std::shared_ptr<VPEWorld::Body> createAndAddBody(glmvec3 scale, glmvec3 position, glmquat orientation, real inv_mass, bool gravity, real friction = 1.0_real);
		std::shared_ptr<VPEWorld::Body> createAndAddWheel(glmvec3 scale, glmvec3 position, glmquat orientation, real inv_mass, bool gravity, real friction = 1.0_real);
		std::shared_ptr<VPEWorld::Body> createAndAddHandlesPassive(glmvec3 scale, glmvec3 position, glmquat orientation, real inv_mass, bool gravity, real friction = 1.0_real);
		glm::vec3 getFramePosition() const;
		std::shared_ptr<VPEWorld::Body> createAndAddHandlesActive(glmvec3 scale, glmvec3 position, glmquat orientation, real inv_mass, bool gravity, real friction = 1.0_real);
		std::shared_ptr<VPEWorld::Body> createAndAddBiker(glmvec3 scale, glmvec3 position, glmquat orientation, real inv_mass, bool gravity, real friction = 1.0_real);
		void setEngine(bool on);
		/// <summary>
		/// Creates a wheel-like structure with one center cube at init_pos and num_cubes rotated equally around it
		/// </summary>
		/// <param name="init_pos"></param>
		/// <param name="num_cubes"></param>
		/// <returns>Returns a pointer to the center cube's body</returns>
		/// 
		void jump();

		void bikeTwoWheels();

	};
}


