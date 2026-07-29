#pragma once

#include <functional>
#include <memory>

#define GLM_ENABLE_EXPERIMENTAL
#include "glm/glm.hpp"
#include "glm/gtc/quaternion.hpp"
#include "glm/gtx/matrix_cross_product.hpp"
#include "glm/gtx/matrix_operation.hpp"
#include "glm/gtx/quaternion.hpp"

import VEPhysicsEngine;

namespace ve {

/// Creates the rigid-body constraint demonstrations used by the example GUI.
class ConstraintDemos {
public:
	using create_visual_callback = std::function<void *(glmvec3, glmvec3, glmquat)>;
	using camera_callback = std::function<glmvec3()>;

	ConstraintDemos(vpe::VPEWorld *physics, vpe::VPEWorld::callback_move on_move,
						 vpe::VPEWorld::callback_erase on_erase, create_visual_callback create_visual,
						 camera_callback camera_position, camera_callback camera_direction)
		: m_physics{physics}, m_onMove{std::move(on_move)}, m_onErase{std::move(on_erase)},
		  m_createVisual{std::move(create_visual)}, m_cameraPosition{std::move(camera_position)},
		  m_cameraDirection{std::move(camera_direction)} {}

	std::shared_ptr<vpe::VPEWorld::Body> createAndAddCube(
		glmvec3 scale, glmvec3 position, glmquat orientation, real inv_mass, bool gravity,
		real friction = 1.0_real);

	std::shared_ptr<vpe::VPEWorld::Body> createWheel(glmvec3 init_pos, int num_cubes);

	void bridge();
	void hingeJoint();
	void ballSocketJoint();
	void wheel();
	void fixedJoint();
	void sliderCannon();
	void hingeChain();
	void sliderJoint();
	void ragdoll();

private:
	vpe::VPEWorld *m_physics;
	vpe::VPEWorld::callback_move m_onMove;
	vpe::VPEWorld::callback_erase m_onErase;
	create_visual_callback m_createVisual;
	camera_callback m_cameraPosition;
	camera_callback m_cameraDirection;
};

} // namespace ve
