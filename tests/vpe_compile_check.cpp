#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

import VEPhysicsEngine;

namespace {
	using Cloth = vpe::VPEWorld::Cloth;

	template <typename Exception = std::exception, typename Function>
	bool throws(Function&& function)
	{
		try
		{
			std::forward<Function>(function)();
		}
		catch (const Exception&)
		{
			return true;
		}
		catch (...)
		{
		}
		return false;
	}

	bool check(bool condition, const char* message)
	{
		if (!condition)
			std::cerr << message << '\n';
		return condition;
	}
}

int main()
{
	static_assert(!std::is_copy_constructible_v<Cloth>);
	static_assert(!std::is_move_constructible_v<Cloth>);

	bool success = true;
	vpe::VPEWorld physics;
	int owner = 0;
	int erased = 0;
	std::vector<glmvec3> vertices{
		{ 0, 1, 0 },
		{ 1, 1, 0 },
		{ 0, 1, 1 }
	};
	std::vector<uint32_t> indices{ 0, 1, 2 };

	auto cloth = std::make_shared<Cloth>(
		&physics, "test cloth", &owner, vpe::VPEWorld::callback_move_cloth{},
		[&erased](const std::shared_ptr<Cloth>&) { ++erased; }, vertices, indices,
		std::vector<glmvec3>{ { 0.0000005, 1, 0 } }, 0._real, 4, 0.8_real);
	physics.addCloth(cloth);
	success &= check(physics.getCloth(&owner) == cloth, "getCloth did not return the stored cloth");
	success &= check(throws<std::invalid_argument>([&] { physics.addCloth(cloth); }),
		"addCloth accepted a duplicate owner");
	success &= check(throws<std::invalid_argument>([&]
		{ physics.addCloth(std::shared_ptr<Cloth>{}); }), "addCloth accepted null");
	physics.eraseCloth(cloth);
	success &= check(physics.m_cloths.empty(), "eraseCloth did not erase from m_cloths");
	success &= check(erased == 1, "eraseCloth did not invoke its callback exactly once");
	success &= check(throws<std::out_of_range>([&] { physics.getCloth(&owner); }),
		"getCloth silently inserted a missing cloth");

	auto makeCloth = [&](std::vector<glmvec3> meshVertices,
		std::vector<uint32_t> meshIndices, int substeps = 4)
	{
		return std::make_shared<Cloth>(
			&physics, "invalid cloth", &owner, vpe::VPEWorld::callback_move_cloth{},
			vpe::VPEWorld::callback_erase_cloth{}, std::move(meshVertices),
			std::move(meshIndices), std::vector<glmvec3>{}, 0._real, substeps, 0.8_real);
	};

	success &= check(throws<std::invalid_argument>([&]
		{ makeCloth({}, indices); }), "empty cloth vertices were accepted");
	success &= check(throws<std::invalid_argument>([&]
		{ makeCloth(vertices, { 0, 1 }); }), "incomplete cloth indices were accepted");
	success &= check(throws<std::out_of_range>([&]
		{ makeCloth(vertices, { 0, 1, 3 }); }), "out-of-range cloth index was accepted");
	success &= check(throws<std::invalid_argument>([&]
		{ makeCloth(vertices, indices, 0); }), "zero cloth substeps were accepted");
	success &= check(throws<std::invalid_argument>([&]
		{ makeCloth({ { 0, 0, 0 }, { 1, 0, 0 }, { 2, 0, 0 } }, indices); }),
		"zero-area cloth triangle was accepted");
	success &= check(throws<std::invalid_argument>([&]
		{ makeCloth({ { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 }, { 2, 2, 2 } }, indices); }),
		"disconnected cloth mass point was accepted");

	vpe::VPEWorld::ClothMassPoint damped({ 0, 1, 0 });
	damped.vel = { -1, 0, 0 };
	damped.damp(1._real, 0.1_real, 0._real);
	success &= check(damped.vel.x < 0._real && damped.vel.x > -1._real,
		"damping did not reduce a negative velocity");

	vpe::VPEWorld::ClothMassPoint grounded({ 0, -1, 0 });
	grounded.vel = { 1, -1, 0 };
	grounded.resolveGroundCollision(1._real);
	success &= check(grounded.pos.y > 0._real, "ground collision did not lift the point");
	success &= check(grounded.vel.x >= 0._real && grounded.vel.x <= 1._real,
		"friction reversed the point velocity");

	vpe::VPEWorld::ClothMassPoint first({ 0, 0, 0 });
	vpe::VPEWorld::ClothMassPoint second({ 1, 0, 0 });
	first.invMass = 1._real;
	second.invMass = 1._real;
	vpe::VPEWorld::ClothConstraint constraint(&first, &second, false);
	second.pos = first.pos;
	constraint.solve(1._real / 60._real, 0._real);
	success &= check(std::isfinite(first.pos.x) && std::isfinite(second.pos.x),
		"zero-length constraint produced a non-finite position");

	vpe::VPEWorld collisionPhysics;
	int collisionClothOwner = 0;
	int collisionBodyOwner = 0;
	std::vector<glmvec3> collisionVertices{
		{ -2, 5, 0 },
		{ -2, 1, 0 },
		{ 2, 5, 0 },
		{ 2, 1, 0 }
	};
	std::vector<uint32_t> collisionIndices{ 0, 1, 2, 2, 1, 3 };
	auto collisionCloth = std::make_shared<Cloth>(
		&collisionPhysics, "collision cloth", &collisionClothOwner,
		vpe::VPEWorld::callback_move_cloth{}, vpe::VPEWorld::callback_erase_cloth{},
		collisionVertices, collisionIndices,
		std::vector<glmvec3>{ collisionVertices[0], collisionVertices[2] },
		0._real, 8, 0.8_real);
	collisionPhysics.addCloth(collisionCloth);
	auto projectile = std::make_shared<vpe::VPEWorld::Body>(
		&collisionPhysics, "projectile", &collisionBodyOwner, &collisionPhysics.g_cube,
		glmvec3{ 1._real }, glmvec3{ 0._real, 3._real, -1.2_real },
		glmquat{ 1._real, 0._real, 0._real, 0._real },
		glmvec3{ 0._real, 0._real, 30._real }, glmvec3{ 0._real },
		0.01_real, 0._real, 1._real);
	collisionPhysics.addBody(projectile);
	for (int frame = 0; frame < 5; ++frame)
		collisionPhysics.tick(1.0 / 60.0);
	const std::vector<glmvec3> deformedVertices = collisionCloth->generateVertices();
	const real impactSurfaceZ =
		(deformedVertices[1].z + deformedVertices[2].z) * 0.5_real;
	const real projectileFrontZ =
		projectile->m_positionW.z + projectile->m_scale.z * 0.5_real;
	success &= check(projectileFrontZ < impactSurfaceZ,
		"fast rigid body crossed the deformed cloth triangle surface");
	success &= check(impactSurfaceZ > 0.1_real,
		"rigid body impact did not deform the attached cloth");
	success &= check(projectile->m_linear_velocityW.z > 1._real &&
		projectile->m_linear_velocityW.z < 29._real,
		"cloth contact did not exchange momentum with the rigid body");
	collisionPhysics.clearCloths();
	collisionPhysics.clear();

	vpe::VPEWorld densePhysics;
	int denseClothOwner = 0;
	int denseBodyOwner = 0;
	constexpr std::size_t denseColumns = 13;
	constexpr std::size_t denseRows = 10;
	std::vector<glmvec3> denseVertices;
	std::vector<glmvec3> denseFixedPoints;
	std::vector<uint32_t> denseIndices;
	for (std::size_t row = 0; row < denseRows; ++row)
		for (std::size_t column = 0; column < denseColumns; ++column)
		{
			denseVertices.push_back({
				4._real * (static_cast<real>(column) /
					static_cast<real>(denseColumns - 1) - 0.5_real),
				3.5_real + 3._real * (0.5_real - static_cast<real>(row) /
					static_cast<real>(denseRows - 1)), 0._real });
			if (row == 0 && (column == 0 || column + 1 == denseColumns))
				denseFixedPoints.push_back(denseVertices.back());
		}
	for (std::size_t row = 0; row + 1 < denseRows; ++row)
		for (std::size_t column = 0; column + 1 < denseColumns; ++column)
		{
			const uint32_t topLeft = static_cast<uint32_t>(row * denseColumns + column);
			const uint32_t topRight = topLeft + 1;
			const uint32_t bottomLeft =
				static_cast<uint32_t>((row + 1) * denseColumns + column);
			const uint32_t bottomRight = bottomLeft + 1;
			denseIndices.insert(denseIndices.end(), {
				topLeft, bottomLeft, topRight, topRight, bottomLeft, bottomRight });
		}
	auto denseCloth = std::make_shared<Cloth>(
		&densePhysics, "dense collision cloth", &denseClothOwner,
		vpe::VPEWorld::callback_move_cloth{}, vpe::VPEWorld::callback_erase_cloth{},
		denseVertices, denseIndices, denseFixedPoints, 0.0005_real, 8, 0.8_real);
	densePhysics.addCloth(denseCloth);
	auto denseProjectile = std::make_shared<vpe::VPEWorld::Body>(
		&densePhysics, "dense projectile", &denseBodyOwner, &densePhysics.g_cube,
		glmvec3{ 1._real }, glmvec3{ 0._real, 3.5_real, -1.2_real },
		glmquat{ 1._real, 0._real, 0._real, 0._real },
		glmvec3{ 0._real, 0._real, 30._real }, glmvec3{ 0._real },
		0.01_real, 0._real, 1._real);
	densePhysics.addBody(denseProjectile);
	for (int frame = 0; frame < 60; ++frame)
		densePhysics.tick(1.0 / 60.0);
	success &= check(fabs(denseProjectile->m_positionW.x) < 0.1_real,
		"dense cloth collision introduced lateral projectile drift");
	success &= check(fabs(denseProjectile->m_positionW.y - 3.5_real) < 0.1_real,
		"dense cloth collision made the projectile jump vertically");
	success &= check(fabs(denseProjectile->m_linear_velocityW.x) < 0.1_real,
		"dense cloth collision introduced lateral projectile shaking");
	success &= check(fabs(denseProjectile->m_linear_velocityW.y) < 0.1_real,
		"dense cloth collision introduced vertical projectile shaking");
	densePhysics.clearCloths();
	densePhysics.clear();

	physics.tick(1.0 / 60.0);
	return success ? 0 : 1;
}
