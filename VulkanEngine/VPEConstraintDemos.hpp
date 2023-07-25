#pragma once

#include "VEInclude.h"
#include "VPE.hpp"

using namespace vpe;

namespace ve {
	/// <summary>
	/// Class that implements a few demos for constraints
	/// I put all the code in here to not clutter main.cpp and the keybinds too much
	/// </summary>
	class ConstraintDemos {
		VPEWorld* m_physics;
		VPEWorld::callback_move m_onMove;
		VPEWorld::callback_erase m_onErase;

	public:
		ConstraintDemos(VPEWorld* physics, VPEWorld::callback_move onMove, VPEWorld::callback_erase onErase) : m_physics{ physics }, m_onMove{ onMove }, m_onErase{ onErase } {}
		~ConstraintDemos() {}

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
		std::shared_ptr<VPEWorld::Body> createAndAddCube(glmvec3 scale, glmvec3 position, glmquat orientation, real inv_mass, bool gravity, real friction = 1.0_real);

		/// <summary>
		/// Creates a wheel-like structure with one center cube at init_pos and num_cubes rotated equally around it
		/// </summary>
		/// <param name="init_pos"></param>
		/// <param name="num_cubes"></param>
		/// <returns>Returns a pointer to the center cube's body</returns>
		std::shared_ptr<VPEWorld::Body> createWheel(glmvec3 init_pos, int num_cubes);

		/// <summary>
		/// Spawns a bridge using distance constraints
		/// </summary>
		void bridge();
		/// <summary>
		/// Spawns a simple hinge joint
		/// </summary>
		void hingeJoint();
		/// <summary>
		/// Spawns a simple ball-and-socket joint
		/// </summary>
		void ballSocketJoint();
		/// <summary>
		/// Spawns a automatically moving wheel using hinge and fixed joints
		/// The motor will revert every 10 seconds
		/// </summary>
		void wheel();
		/// <summary>
		/// Spawns a simple fixed joint
		/// </summary>
		void fixedJoint();
		/// <summary>
		/// Spawns a slider cannon. Un-comment the not-thread safe code here if you want to spawn things to shoot away as well
		/// </summary>
		void sliderCannon();
		/// <summary>
		/// Spawns a hinge chain. After some time, a motor will kick in
		/// </summary>
		void hingeChain();
		/// <summary>
		/// Spawns a simple slider joint
		/// </summary>
		void sliderJoint();
		/// <summary>
		/// Spawns a (rather sad) attempt at ragdoll physics
		/// </summary>
		void ragdoll();
	};
}

