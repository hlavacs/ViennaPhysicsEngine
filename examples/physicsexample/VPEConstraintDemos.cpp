#include "VPEConstraintDemos.hpp"

using namespace vpe;

namespace ve {
	std::shared_ptr<VPEWorld::Body> ConstraintDemos::createAndAddCube(glmvec3 scale, glmvec3 position, glmquat orientation, real inv_mass, bool gravity, real friction) {
		VESceneNode* cube;
		VECHECKPOINTER(cube = getSceneManagerPointer()->loadModel("The Cube" + std::to_string(m_physics->m_body_id), "../../media/models/test/crate0", "cube.obj", 0, getRoot()));
		auto body = std::make_shared<VPEWorld::Body>(m_physics, "Body" + std::to_string(m_physics->m_bodies.size()), cube, &m_physics->g_cube, scale, position, orientation, glmvec3{ 0.0_real }, glmvec3{ 0.0_real }, inv_mass, m_physics->m_restitution, friction);
		body->m_on_move = m_onMove;
		body->m_on_erase = m_onErase;
		m_physics->addBody(body);
		if (gravity) body->setForce(0ul, VPEWorld::Force{ {0, m_physics->c_gravity, 0} });

		return body;
	}

	std::shared_ptr<VPEWorld::Body> ConstraintDemos::createWheel(glmvec3 init_pos, int num_cubes) {
		real constexpr cube_mass = 1.0_real / 100.0_real;
		auto centerBody = createAndAddCube(glmvec3{ 1.0_real }, init_pos, glmquat{ 1, 0, 0, 0 }, cube_mass, true);

		glmvec3 pos = init_pos;
		pos[1] += 3.0_real;
		std::vector<std::shared_ptr<VPEWorld::Body>> bodies;

		glmmat4 rot(1.0_real);
		real rot_amount = pi2 / (num_cubes);

		for (int i = 0; i < num_cubes; ++i) {
			auto body = createAndAddCube(glmvec3{ 1.0_real }, pos, glmquat{ 1, 0, 0, 0 }, cube_mass, true);
			bodies.push_back(body);

			pos -= init_pos;
			pos = glm::rotate(rot_amount, glmvec3(0, 0, 1)) * glmvec4(pos, 1.0_real);
			pos += init_pos;
		}

		for (int i = 0; i < num_cubes; ++i) {
			glmvec3 cube_anchor = (bodies[i]->m_positionW + init_pos) * 0.5_real;
			auto constraint = std::make_shared<VPEWorld::FixedJoint>(centerBody, bodies[i], cube_anchor); // Choosing the second cube as the anchor point should increase stiffness
			m_physics->addConstraint(constraint);
		}

		return centerBody;
	}

	void ConstraintDemos::bridge() {
		glmvec3 positionCamera{ getSceneManagerPointer()->getSceneNode("StandardCameraParent")->getWorldTransform()[3] };
		glmvec3 dir{ getSceneManagerPointer()->getSceneNode("StandardCamera")->getWorldTransform()[2] };

		positionCamera = positionCamera + 2.0_real * dir;
		real cubeMass = 0;
		std::shared_ptr<VPEWorld::Body> prevBody;
		positionCamera[1] += 8.0_real;

		for (int i = 0; i < 10; ++i) {
			if (i == 0 || i == 9) cubeMass = 0;
			else cubeMass = 1.0_real / 100.0_real;

			auto body = createAndAddCube(glmvec3{ 1.0_real }, positionCamera, glmquat(1, 0, 0, 0), cubeMass, !(i == 0 || i == 9));

			if (i > 0) {
				auto constraint = std::make_shared<VPEWorld::DistanceConstraint>(body, prevBody, 2.3_real);
				m_physics->addConstraint(constraint);
			}

			prevBody = body;
			positionCamera[0] += 2.0_real;
		}
	}

	void ConstraintDemos::hingeJoint() {
		glmvec3 positionCamera{ getSceneManagerPointer()->getSceneNode("StandardCameraParent")->getWorldTransform()[3] };
		glmvec3 dir{ getSceneManagerPointer()->getSceneNode("StandardCamera")->getWorldTransform()[2] };
		positionCamera[1] += 2.0_real;

		glmvec3 cubePos1 = positionCamera + 2.0_real * dir;
		glmvec3 cubePos2 = cubePos1;
		cubePos2[0] += 2.0_real;

		glmvec3 jointAnchor = cubePos1;
		glmvec3 jointAxis(0.0_real, 1.0_real, 0.0_real);

		auto body1 = createAndAddCube(glmvec3{ 1.0_real }, cubePos1, glmquat{ 1, 0, 0, 0 }, 0.0_real, false);
		auto body2 = createAndAddCube(glmvec3{ 1.0_real }, cubePos2, glmquat{ 1, 0, 0, 0 }, 1.0_real / 100.0_real, true);

		auto constraint = std::make_shared<VPEWorld::HingeJoint>(body1, body2, jointAnchor, jointAxis);
		constraint->enableLimit(-pi2/3.0_real, pi2/3.0_real);
		m_physics->addConstraint(constraint);
	}

	void ConstraintDemos::ballSocketJoint() {
		glmvec3 positionCamera{ getSceneManagerPointer()->getSceneNode("StandardCameraParent")->getWorldTransform()[3] };
		glmvec3 dir{ getSceneManagerPointer()->getSceneNode("StandardCamera")->getWorldTransform()[2] };
		positionCamera[1] += 3.0_real;

		glmvec3 cubePos1 = positionCamera + 2.0_real * dir;
		glmvec3 cubePos2 = cubePos1;
		cubePos2[0] += 2.0;
		glmvec3 jointAnchor = cubePos1;

		auto body1 = createAndAddCube(glmvec3{ 1.0_real }, cubePos1, glmquat{ 1, 0, 0, 0 }, 0.0_real, false);
		auto body2 = createAndAddCube(glmvec3{ 1.0_real }, cubePos2, glmquat{ 1, 0, 0, 0 }, 1.0_real / 100.0_real, true);

		auto constraint = std::make_shared<VPEWorld::BallSocketJoint>(body1, body2, jointAnchor);
		m_physics->addConstraint(constraint);
	}

	void ConstraintDemos::wheel() {
		glmvec3 positionCamera{ getSceneManagerPointer()->getSceneNode("StandardCameraParent")->getWorldTransform()[3] };
		glmvec3 dir{ getSceneManagerPointer()->getSceneNode("StandardCamera")->getWorldTransform()[2] };

		glmvec3 centerPos = positionCamera + 2.0_real * dir;
		centerPos[1] += 7.0_real;

		auto center1 = createWheel(centerPos, 6);
		centerPos[2] += 8;
		auto center2 = createWheel(centerPos, 6);

		centerPos = 0.5_real * (center1->m_positionW + center2->m_positionW);
		auto centerBody = createAndAddCube(glmvec3{ 1.0_real }, centerPos, glmquat{ 1, 0, 0, 0 }, 1.0_real / 100.0_real, false);

		real constexpr motor_speed = 3.0_real;
		real constexpr motor_max_force = 30.0_real;

		glmvec3 jointAxis{ 0.0_real, 0.0_real, 1.0_real };
		auto constraint1 = std::make_shared<VPEWorld::HingeJoint>(centerBody, center1, centerPos, jointAxis);
		m_physics->addConstraint(constraint1);
		constraint1->enableMotor(motor_speed, motor_max_force);
		constraint1->setBody1MotorEnabled(false);

		auto constraint2 = std::make_shared<VPEWorld::HingeJoint>(centerBody, center2, centerPos, jointAxis);
		m_physics->addConstraint(constraint2);
		constraint2->enableMotor(motor_speed, motor_max_force);
		constraint2->setBody1MotorEnabled(false);

		// We capture copies here, otherwise this stuff will be undefined when the function terminates and it goes out of scope
		std::thread flipMotorThread([constraint1, constraint2, motor_speed, motor_max_force]() {
			int sign = 1;

			while (true) {
				std::this_thread::sleep_for(std::chrono::milliseconds(10000));
				sign *= -1;
				constraint1->enableMotor(sign * motor_speed, motor_max_force);
				constraint2->enableMotor(sign * motor_speed, motor_max_force);
			}
			});

		flipMotorThread.detach();
	}

	void ConstraintDemos::fixedJoint() {
		glmvec3 positionCamera{ getSceneManagerPointer()->getSceneNode("StandardCameraParent")->getWorldTransform()[3] };
		glmvec3 dir{ getSceneManagerPointer()->getSceneNode("StandardCamera")->getWorldTransform()[2] };
		positionCamera[1] += 3.0_real;

		glmvec3 cubePos1 = positionCamera + 2.0_real * dir;
		glmvec3 cubePos2 = cubePos1;
		cubePos2 += glmvec3(1.0_real);
		glmvec3 jointAnchor = 0.5_real * (cubePos1 + cubePos2);
		jointAnchor = cubePos1;

		auto body1 = createAndAddCube(glmvec3{ 1.0_real }, cubePos1, glmquat{ 1, 0, 0, 0 }, 1.0_real / 100.0_real, true);
		auto body2 = createAndAddCube(glmvec3{ 1.0_real }, cubePos2, glmquat{ 1, 0, 0, 0 }, 1.0_real / 100.0_real, true);

		auto constraint = std::make_shared<VPEWorld::FixedJoint>(body1, body2, jointAnchor);
		m_physics->addConstraint(constraint);
	}

	void ConstraintDemos::sliderCannon() {
		glmvec3 positionCamera{ getSceneManagerPointer()->getSceneNode("StandardCameraParent")->getWorldTransform()[3] };
		glmvec3 dir{ getSceneManagerPointer()->getSceneNode("StandardCamera")->getWorldTransform()[2] };

		glmvec3 cubePos1 = positionCamera + 2.0_real * dir;
		glmvec3 cubePos2 = cubePos1;
		cubePos2[2] += 1.0_real;
		cubePos2[1] += 1.0_real;

		glmvec3 jointAnchor = cubePos2;

		//  glmquat{ 0.9238796_real, 0.3826834_real, 0, 0 } // 45 degrees around x axis
		auto body1 = createAndAddCube(glmvec3{ 1.0_real }, cubePos1, glmquat{ 0.9238796_real, 0.3826834_real, 0, 0 }, 0.0_real / 100.0_real, false);
		auto body2 = createAndAddCube(glmvec3{ 1.0_real }, cubePos2, glmquat{ 0.9238796_real, 0.3826834_real, 0, 0 }, 1.0_real / 100.0_real, true);

		glmvec3 stackPos = cubePos2;
		stackPos[1] = 0.9_real + positionCamera[1] + 2.0_real * dir[1] - 1.0_real;
		stackPos[2] += 1.9_real;

		auto stack1 = createAndAddCube(glmvec3{ 1.0_real }, stackPos, glmquat{ 1, 0, 0, 0 }, 0.0_real / 100.0_real, false);
		stackPos[1] += 1.0_real;

		auto stack2 = createAndAddCube(glmvec3{ 1.0_real }, stackPos, glmquat{ 1, 0, 0, 0 }, 0.0_real / 100.0_real, false);
		stackPos[1] += 1.0_real;

		glmvec3 jointAxis{ 0.0_real, 1.0_real, 1.0_real };
		auto constraint = std::make_shared<VPEWorld::SliderJoint>(body1, body2, jointAnchor, jointAxis);
		m_physics->addConstraint(constraint);
		constraint->enableLimit(-1, 10);

		auto physics = m_physics;
		// We capture copies here, otherwise this stuff will be undefined when the function terminates and it goes out of scope
		std::thread motorThread([this, constraint, stackPos, physics]() {
			auto stackPosLeft = stackPos; stackPosLeft[0] -= 2;
			auto stackPosRight = stackPos; stackPosRight[0] += 2;

			while (true) {
				std::this_thread::sleep_for(std::chrono::milliseconds(3000));
				// THIS IS NOT THREADSAFE AT ALL, USE AT OWN RISK
				// Will eventually crash the program due to iterator invalidation, especially when compiling in debug mode
				// Also doesn't work with the engine's debug mode
				/*
				auto stackCenter = createAndAddCube(glmvec3{ 1.0_real }, stackPos, glmquat{ 1, 0, 0, 0 }, 1.0_real / 100.0_real, true);
				auto stackLeft = createAndAddCube(glmvec3{ 1.0_real }, stackPosLeft, glmquat{ 1, 0, 0, 0 }, 6.0_real / 100.0_real, true);
				auto stackRight = createAndAddCube(glmvec3{ 1.0_real }, stackPosRight, glmquat{ 1, 0, 0, 0 }, 6.0_real / 100.0_real, true);

				auto constraint1 = std::make_shared<VPEWorld::FixedJoint>(stackCenter, stackLeft, (stackPos + stackPosLeft) * 0.5_real);
				auto constraint2 = std::make_shared<VPEWorld::FixedJoint>(stackCenter, stackRight, (stackPos + stackPosRight) * 0.5_real);
				m_physics->addConstraint(constraint1);
				m_physics->addConstraint(constraint2);
				*/

				std::this_thread::sleep_for(std::chrono::milliseconds(700));
				constraint->enableMotor(5000, 100000);
				// Disabling the motor does not remove the velocity (neither does hitting the limit point), so go backwards here
				std::this_thread::sleep_for(std::chrono::milliseconds(400));
				constraint->enableMotor(-200, 5000);;
			}
		});

		motorThread.detach();
	}

	void ConstraintDemos::hingeChain() {
		glmvec3 positionCamera{ getSceneManagerPointer()->getSceneNode("StandardCameraParent")->getWorldTransform()[3] };
		glmvec3 dir{ getSceneManagerPointer()->getSceneNode("StandardCamera")->getWorldTransform()[2] };

		glmvec3 pos = positionCamera + 2.0_real * dir;
		pos[1] += 14.0_real;

		glmvec3 prevPos = pos;
		int num_cubes = 8;
		std::vector<std::shared_ptr<VPEWorld::Body>> bodies;

		for (int i = 0; i < num_cubes; ++i) {
			real cube_mass = (i == 0) ? 0.0_real : 1.0_real / 100.0_real;

			auto body = createAndAddCube(glmvec3{ 1.0_real }, pos, glmquat{ 1, 0, 0, 0 }, cube_mass, !(i == 0));
			bodies.push_back(body);

			prevPos = pos;
			pos[2] += 1.5_real;
		}

		std::shared_ptr<VPEWorld::HingeJoint> mainConstraint;
		glmvec3 hinge_axis(1, 0, 0);
		for (int i = 1; i < num_cubes; ++i) {
			auto body1 = bodies[i - 1];
			auto body2 = bodies[i];
			glmvec3 position = body1->m_positionW;
			auto constraint = std::make_shared<VPEWorld::HingeJoint>(body1, body2, position, hinge_axis);
			if (i == 1) mainConstraint = constraint;
			m_physics->addConstraint(constraint);
		}

		std::thread enableMotorThread([mainConstraint]() {
			while (true) {
				std::this_thread::sleep_for(std::chrono::milliseconds(10000));
				mainConstraint->enableMotor(7.0_real, 25.0_real);
			}
			});

		enableMotorThread.detach();
	}

	void ConstraintDemos::sliderJoint() {
		glmvec3 positionCamera{ getSceneManagerPointer()->getSceneNode("StandardCameraParent")->getWorldTransform()[3] };
		glmvec3 dir{ getSceneManagerPointer()->getSceneNode("StandardCamera")->getWorldTransform()[2] };

		glmvec3 cubePos1 = positionCamera + 2.0_real * dir;
		glmvec3 cubePos2 = cubePos1;
		cubePos2[2] += 1.5_real;
		glmvec3 jointAnchor = cubePos2;

		auto body1 = createAndAddCube(glmvec3{ 1.0_real }, cubePos1, glmquat{ 1, 0, 0, 0 }, 0.0_real / 100.0_real, false);
		auto body2 = createAndAddCube(glmvec3{ 1.0_real }, cubePos2, glmquat{ 1, 0, 0, 0 }, 1.0_real / 100.0_real, true);

		glmvec3 jointAxis{ 0.0_real, 0.0_real, 1.0_real };
		auto constraint = std::make_shared<VPEWorld::SliderJoint>(body1, body2, jointAnchor, jointAxis);
		constraint->enableLimit(-3.0_real, 15.0_real);
		m_physics->addConstraint(constraint);
	}

	void ConstraintDemos::ragdoll() {
		glmvec3 positionCamera{ getSceneManagerPointer()->getSceneNode("StandardCameraParent")->getWorldTransform()[3] };
		glmvec3 dir{ getSceneManagerPointer()->getSceneNode("StandardCamera")->getWorldTransform()[2] };

		glmvec3 centerPos = positionCamera + 2.0_real * dir;
		centerPos[1] += 2;
		auto torso = createAndAddCube(glmvec3{ 0.6_real, 1.0_real, 0.4_real }, centerPos, glmquat{ 1, 0, 0, 0 }, 1.0_real / 100.0_real, true, 0.2_real);

		glmvec3 cubePos = centerPos;
		cubePos[0] -= 0.75_real;
		cubePos[1] += 0.85_real;
		auto arm1 = createAndAddCube(glmvec3{ 0.2_real, 0.7_real, 0.2_real }, cubePos, glmquat{ 0.9238796_real, 0, 0, 0.3826834_real }, 4.0_real / 100.0_real, true, 0.2_real);
		cubePos[0] += 1.5_real;
		auto arm2 = createAndAddCube(glmvec3{ 0.2_real, 0.7_real, 0.2_real }, cubePos, glmquat{ 0.9238796_real, 0, 0, -0.3826834_real }, 4.0_real / 100.0_real, true, 0.2_real);
		cubePos = centerPos;
		cubePos[0] -= 0.8_real;
		cubePos[1] -= 0.9_real;
		auto leg1 = createAndAddCube(glmvec3{ 0.2_real, 1.0_real, 0.2_real }, cubePos, glmquat{ 0.9238796_real, 0, 0, -0.3826834_real }, 4.0_real / 100.0_real, true, 0.2_real);
		cubePos[0] += 1.6_real;
		auto leg2 = createAndAddCube(glmvec3{ 0.2_real, 1.0_real, 0.2_real }, cubePos, glmquat{ 0.9238796_real, 0, 0, 0.3826834_real }, 4.0_real / 100.0_real, true, 0.2_real);

		cubePos = centerPos; cubePos[1] += 0.9_real;
		auto head = createAndAddCube(glmvec3{ 0.6_real, 0.6_real, 0.4_real }, cubePos, glmquat{ 1, 0, 0, 0 }, 3.0_real / 100.0_real, true, 0.2_real);

		auto torsoHead = std::make_shared<VPEWorld::FixedJoint>(torso, head, head->m_positionW);
		m_physics->addConstraint(torsoHead);

		auto torsoArm1 = std::make_shared<VPEWorld::BallSocketJoint>(torso, arm1, centerPos + glmvec3(-0.4_real, 0.5_real, 0.0_real));
		m_physics->addConstraint(torsoArm1);

		auto torsoArm2 = std::make_shared<VPEWorld::BallSocketJoint>(torso, arm2, centerPos + glmvec3(0.4_real, 0.5_real, 0.0_real));
		m_physics->addConstraint(torsoArm2);

		auto torsoLeg1 = std::make_shared<VPEWorld::BallSocketJoint>(torso, leg1, centerPos + glmvec3(-0.2_real, -0.45_real, 0.0_real));
		m_physics->addConstraint(torsoLeg1);

		auto torsoLeg2 = std::make_shared<VPEWorld::BallSocketJoint>(torso, leg2, centerPos + glmvec3(0.2_real, -0.45_real, 0.0_real));
		m_physics->addConstraint(torsoLeg2);
	}
}