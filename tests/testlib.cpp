#include "VPE.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

namespace {

std::shared_ptr<vpe::VPEWorld::Body> makeBody(vpe::VPEWorld& world, void* owner, glmvec3 position, real mass_inv = 1.0_real) {
	return std::make_shared<vpe::VPEWorld::Body>(
		&world,
		"body",
		owner,
		&world.g_cube,
		glmvec3{ 1.0_real },
		position,
		glmquat{ 1, 0, 0, 0 },
		glmvec3{ 0.0_real },
		glmvec3{ 0.0_real },
		mass_inv,
		world.m_restitution,
		world.m_friction);
}

void testBodyLifecycle() {
	vpe::VPEWorld world;
	int owner = 0;
	auto body = makeBody(world, &owner, glmvec3{ 0.0_real, 2.0_real, 0.0_real });

	world.addBody(body);

	assert(world.m_bodies.size() == 1);
	assert(world.getBody(&owner).second == body);
	assert(world.m_body == body);

	world.eraseBody(body);

	assert(world.m_bodies.size() == 0);
	const intpair_t cell{ body->m_grid_x, body->m_grid_z };
	assert(world.m_grid[cell].size() == 0);
}

void testGravityIntegration() {
	vpe::VPEWorld world;
	world.m_mode = vpe::VPEWorld::SIMULATION_MODE_DEBUG;
	world.m_current_time = world.m_sim_delta_time * 1.5;

	int owner = 0;
	auto body = makeBody(world, &owner, glmvec3{ 0.0_real, 10.0_real, 0.0_real });
	body->setForce(0, vpe::VPEWorld::Force{ { 0.0_real, world.c_gravity, 0.0_real } });
	world.addBody(body);

	world.tick(0.0);

	assert(body->m_linear_velocityW.y < 0.0_real);
	assert(body->m_positionW.y < 10.0_real);
}

void testConstraintSetup() {
	vpe::VPEWorld world;
	int owner1 = 0;
	int owner2 = 0;
	auto body1 = makeBody(world, &owner1, glmvec3{ 0.0_real, 2.0_real, 0.0_real });
	auto body2 = makeBody(world, &owner2, glmvec3{ 2.0_real, 2.0_real, 0.0_real });

	world.addBody(body1);
	world.addBody(body2);

	auto constraint = std::make_shared<vpe::VPEWorld::DistanceConstraint>(body1, body2, 2.0_real);
	world.addConstraint(constraint);
	world.setupConstraints(world.m_sim_delta_time);
	world.calculateImpulses(1, world.m_sim_delta_time);

	assert(world.m_constraints.size() == 1);
	world.removeConstraint(constraint);
	assert(world.m_constraints.empty());
}

void testClothLifecycle() {
	vpe::VPEWorld world;
	int owner = 0;
	bool erased = false;

	std::vector<glmvec3> vertices{
		{ 0.0_real, 2.0_real, 0.0_real },
		{ 1.0_real, 2.0_real, 0.0_real },
		{ 0.0_real, 3.0_real, 0.0_real },
	};
	std::vector<uint32_t> indices{ 0, 1, 2 };
	std::vector<glmvec3> fixedPoints{ vertices[0] };

	auto cloth = std::make_shared<vpe::VPEWorld::Cloth>(
		&world,
		"cloth",
		&owner,
		nullptr,
		[&](std::shared_ptr<vpe::VPEWorld::Cloth>) { erased = true; },
		vertices,
		indices,
		fixedPoints,
		1.0_real,
		1,
		0.8_real);

	world.addCloth(cloth);
	assert(world.m_cloths.size() == 1);
	assert(world.getCloth(&owner) == cloth);

	world.eraseCloth(cloth);

	assert(erased);
	assert(world.m_cloths.empty());
	assert(world.m_bodies.empty());
}

} // namespace

int main() {
	testBodyLifecycle();
	testGravityIntegration();
	testConstraintSetup();
	testClothLifecycle();

	std::cout << "All VPE core tests passed.\n";
	return 0;
}
