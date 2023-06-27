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

		void bridge();
		void hingeJoint();
		void ballSocketJoint();
		void wheel();
		void fixedJoint();
		void sliderCannon();
		void hingeChain();
		void sliderJoint();
		void ragdoll();
	};
}