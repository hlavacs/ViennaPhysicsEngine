#include "TrialBike.hpp"
#ifndef DEBUG_BIKE
#define DEBUG_BIKE 1
#endif

using namespace vpe;

namespace ve {
    //"../../media/models/trial-bike", "bike_body.obj"
	std::shared_ptr<VPEWorld::Body> TrialBike::createAndAddBody(glmvec3 scale, glmvec3 position, glmquat orientation, real inv_mass, bool gravity, real friction) {
		VESceneNode* model;
		VECHECKPOINTER(model = getSceneManagerPointer()->loadModel("Model" + std::to_string(m_physics->m_body_id), "../../media/models/trial-bike", "bike_body.obj", 0, getRoot()));
		auto body = std::make_shared<VPEWorld::Body>(m_physics, "Body" + std::to_string(m_physics->m_bodies.size()), model, &m_physics->g_cube, scale, position, orientation, glmvec3{ 0.0_real }, glmvec3{ 0.0_real }, inv_mass, m_physics->m_restitution, friction);
		body->m_on_move = m_onMove;
		body->m_on_erase = m_onErase;
		m_physics->addBody(body);
		if (gravity) body->setForce(0ul, VPEWorld::Force{ {0, m_physics->c_gravity, 0} });

		return body;
	}

	std::shared_ptr<VPEWorld::Body> TrialBike::createAndAddWheel(glmvec3 scale, glmvec3 position, glmquat orientation, real inv_mass, bool gravity, real friction) {
        VESceneNode* model;
        // "../../media/models/test/crate0", "cube.obj"
        // "../../media/models/trial-bike", "tire.obj"
		VECHECKPOINTER(model = getSceneManagerPointer()->loadModel("Model" + std::to_string(m_physics->m_body_id), "../../media/models/trial-bike", "tire.obj", 0, getRoot()));
		auto body = std::make_shared<VPEWorld::Body>(m_physics, "Body" + std::to_string(m_physics->m_bodies.size()), model, &m_physics->g_cylinder, scale, position, orientation, glmvec3{ 0.0_real }, glmvec3{ 0.0_real }, inv_mass, m_physics->m_restitution, friction);
		body->m_on_move = m_onMove;
		body->m_on_erase = m_onErase;
		m_physics->addBody(body);
		if (gravity) body->setForce(0ul, VPEWorld::Force{ {0, m_physics->c_gravity, 0} });

		return body;
	}

    std::shared_ptr<VPEWorld::Body> TrialBike::createAndAddHandlesPassive(glmvec3 scale, glmvec3 position, glmquat orientation, real inv_mass, bool gravity, real friction) {
        VESceneNode* model;
        // "../../media/models/test/crate0", "cube.obj"
        // "../../media/models/trial-bike", "tire.obj"
        VECHECKPOINTER(model = getSceneManagerPointer()->loadModel("Model" + std::to_string(m_physics->m_body_id), "../../media/models/trial-bike", "handles_passive.obj", 0, getRoot()));
        auto body = std::make_shared<VPEWorld::Body>(m_physics, "Body" + std::to_string(m_physics->m_bodies.size()), model, &m_physics->g_cylinder, scale, position, orientation, glmvec3{ 0.0_real }, glmvec3{ 0.0_real }, inv_mass, m_physics->m_restitution, friction);
        body->m_on_move = m_onMove;
        body->m_on_erase = m_onErase;
        m_physics->addBody(body);
        if (gravity) body->setForce(0ul, VPEWorld::Force{ {0, m_physics->c_gravity, 0} });

        return body;
    }

    void TrialBike::bikeTwoWheels()
    {
        // bike a couple of metres in front of the camera 
        glmvec3 camPos{ getSceneManagerPointer()->getSceneNode("StandardCameraParent")->getWorldTransform()[3] };
        glmvec3 fwd{ getSceneManagerPointer()->getSceneNode("StandardCamera")->getWorldTransform()[2] };

        // bike geometry 
        const real wheelbase = 3.0_real;   // distance between axles
        const real axleHeight = 1.0_real;   // height above ground
        const real invMassFrame = 1.0_real / 100.0_real;
        const real invMassWheel = 1.0_real / 15.0_real;
        const real wheelFriction = 1.2_real;

        // hinge axis
        const glmvec3 hingeAxis{ 1.0_real, 0.0_real, 0.0_real };

        // motor setup 
        const real targetOmega = 6.0_real;
        const real maxMotorTorque = 80.0_real;

        // centre of the bike, between axles
        glmvec3 centre = camPos + 2.0_real * fwd;
        centre[1] = axleHeight;

        // rear and front axle positions along the forward vector
        glmvec3 rearPos = centre - 0.5_real * wheelbase * glm::normalize(fwd);
        glmvec3 frontPos = centre + 0.5_real * wheelbase * glm::normalize(fwd);

        centre[1] = 2.0_real;


        // create bodies
        auto frame = TrialBike::createAndAddBody(glmvec3{ 1.0_real }, centre, glmquat{ 1,0,0,0 }, invMassFrame, true, 0.6_real);

        glmquat wheelOri{ 1,0,0,0 };


        auto rearWheel = TrialBike::createAndAddWheel(glmvec3{ 1.0_real }, rearPos, wheelOri, invMassWheel, true, wheelFriction);
        auto frontWheel = TrialBike::createAndAddWheel(glmvec3{ 1.0_real }, frontPos, wheelOri, invMassWheel, true, wheelFriction);


        // hinge each wheel to the frame about the axle
        auto rearHinge = std::make_shared<VPEWorld::HingeJoint>(frame, rearWheel, rearPos, hingeAxis);
        auto frontHinge = std::make_shared<VPEWorld::HingeJoint>(frame, frontWheel, frontPos, hingeAxis);

        // drive rear wheel continuously in one direction
        rearHinge->enableMotor(targetOmega, maxMotorTorque);
        rearHinge->setBody1MotorEnabled(false);   // motor acts on body2


        m_physics->addConstraint(rearHinge);
        m_physics->addConstraint(frontHinge);


        // handle positions
        glmvec3 handlePos = centre + glmvec3{ 0.0_real, 0.3_real, 0.9_real };  // Position in front of the body (adjust as necessary)
        glmquat handleOri{ 1, 0, 0, 0 }; // Default orientation

        // create passive handles
        auto handles = TrialBike::createAndAddHandlesPassive(glmvec3{ 1.0_real }, handlePos, handleOri, 0.1_real, false, 0.5_real);

        const glmvec3 hingeAxis2{ 0.0_real, 1.0_real, 0.0_real };
        // hinge joint
        auto handleHingeJoint = std::make_shared<VPEWorld::HingeJoint>(frame, handles, handlePos, hingeAxis2);

        // limit the rotation
        handleHingeJoint->enableLimit(-pi2 / 4.0_real, pi2 / 4.0_real);

        m_physics->addConstraint(handleHingeJoint);

    }



}
