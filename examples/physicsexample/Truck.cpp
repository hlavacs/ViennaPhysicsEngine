#include "Truck.hpp"
#ifndef DEBUG_Truck
#define DEBUG_Truck 1
#endif

using namespace vpe;

namespace ve {
    std::shared_ptr<VPEWorld::Body> Truck::createAndAddBody(glmvec3 scale, glmvec3 position, glmquat orientation, real inv_mass, bool gravity, real friction) {
		
        // Load the truck model
        VESceneNode* model;
        VECHECKPOINTER(model = getSceneManagerPointer()->loadModel("Model" + std::to_string(m_physics->m_body_id), "../../media/models/truck", "Truck.obj", 0, getRoot()));
        
		// Create the truck body with the specified parameters
        auto body = std::make_shared<VPEWorld::Body>(m_physics, "Body" + std::to_string(m_physics->m_bodies.size()), model, &m_physics->g_truck_cube, scale, position, orientation, glmvec3{ 0.0_real }, glmvec3{ 0.0_real }, inv_mass, m_physics->m_restitution, friction);
        
		// Set callbacks for movement and erasure
        body->m_on_move = m_onMove;
        body->m_on_erase = m_onErase;
        
        // Add the body to the physics world
        m_physics->addBody(body);

		// Apply gravity if needed
		
        body->setForce(0ul, VPEWorld::Force{ {0, 0, 0} });
        if (gravity) {
            // For testing purposes, we apply also a horizontal force on the truck body, so it rolls naturally
            body->setForce(0ul, VPEWorld::Force{ {0 , m_physics->c_gravity, -m_physics->c_gravity / 3} });
        }
        return body;
    }
     
    std::shared_ptr<VPEWorld::Body> Truck::createAndAddWheel(WheelPosition wheelPosition, glmvec3 scale, glmvec3 position, glmquat orientation, real inv_mass, bool gravity, real friction) {
        VESceneNode* model;

        // Determine the model file based on wheel position
        std::string wheelModel;
        switch (wheelPosition) {
        case WheelPosition::FrontLeft:
            wheelModel = "FrontLeftWheel.obj";
            break;
        case WheelPosition::FrontRight:
            wheelModel = "FrontRightWheel.obj";
            break;
        case WheelPosition::RearLeft:
            wheelModel = "RearLeftWheel.obj";
            break;
        case WheelPosition::RearRight:
            wheelModel = "RearRightWheel.obj";
            break;
        default:
            wheelModel = "TruckWheel.obj";
            break;
        }

        // Load the corresponding wheel model
        VECHECKPOINTER(model = getSceneManagerPointer()->loadModel("Model" + std::to_string(m_physics->m_body_id), "../../media/models/truck", wheelModel, 0, getRoot()));

        // Create the wheel body with the specified parameters
        auto body = std::make_shared<VPEWorld::Body>(m_physics, "Body" + std::to_string(m_physics->m_bodies.size()), model, &m_physics->g_Cylinder, scale, position, orientation, glmvec3{ 0.0_real }, glmvec3{ 0.0_real }, inv_mass, m_physics->m_restitution, friction);

        // Set callbacks for movement and erasure
        body->m_on_move = m_onMove;
        body->m_on_erase = m_onErase;

        // Add the body to the physics world
        m_physics->addBody(body);

        // Apply gravity if needed
        body->setForce(0ul, VPEWorld::Force{ {0, 0, 0} });
        if (gravity) {
            body->setForce(0ul, VPEWorld::Force{ {0, m_physics->c_gravity, 0} });
        }

        // Return the created wheel body
        return body;
    }


    void Truck::truckFourWheels() {
        glmvec3 camPos{ getSceneManagerPointer()->getSceneNode("StandardCameraParent")->getWorldTransform()[3] };
        glmvec3 fwd{ getSceneManagerPointer()->getSceneNode("StandardCamera")->getWorldTransform()[2] };

		//----------------------------------------------
		// Truck BODY CREATION
        
		// Truck body parameters
		const bool gravity = true;
        const real invMassFrame = 1.0_real / 500.0_real;  // Inv Truck body mass
		glmvec3 truckCenter = camPos + 30.0_real * fwd; // Start 30 units in front of the camera
        truckCenter.y = 5.0_real;
        
        // Create the truck's body (frame)
        auto frame = Truck::createAndAddBody(glmvec3{ 1.0_real, 1.0_real, 1.0_real }, truckCenter, glmquat{ 1, 0, 0, 0 }, invMassFrame, gravity, 1.0_real);

        //----------------------------------------------
        // Wheels CREATION

		// Wheel parameters
        const real wheelFriction = 1.5_real;  // Higher friction for truck wheels
		glmquat wheelOrientation = glmquat(glmvec3(0.0f, glm::radians(90.0f), 0.0f)); // Rotate wheels to align properly
        const real invMassWheel = 1.0_real / 30.0_real;  // Inv Wheel mass

		// Offsets for tire positions relative to truck center
		// Calculated based on truck model dimensions
        // used blender to find relative positions
        glmvec3 frontRightOffset = glmvec3(-1.72513f , -1.36934f, -1.66935f);
        glmvec3 frontLeftOffset = glmvec3(1.72512f , -1.36934f, -1.66935f);
        glmvec3 rearRightOffset = glmvec3(-1.72209f, -1.36934f, 1.53065f);
        glmvec3 rearLeftOffset = glmvec3(1.72392f, -1.36934f, 1.53065f);

        // Tire positions based on offsets
        glmvec3 frontRightTirePos = truckCenter + frontRightOffset;
        glmvec3 frontLeftTirePos = truckCenter + frontLeftOffset;
        glmvec3 rearRightTirePos = truckCenter + rearRightOffset;
        glmvec3 rearLeftTirePos = truckCenter + rearLeftOffset;

        // Create the wheels using these positions (adjust to your wheel creation logic)
        auto rearLeftWheel = Truck::createAndAddWheel(WheelPosition::RearLeft, glmvec3(1.0f), rearLeftTirePos, wheelOrientation, invMassWheel, gravity, wheelFriction);
        auto rearRightWheel = Truck::createAndAddWheel(WheelPosition::RearRight, glmvec3(1.0f), rearRightTirePos, wheelOrientation, invMassWheel, gravity, wheelFriction);
        auto frontLeftWheel = Truck::createAndAddWheel(WheelPosition::FrontLeft, glmvec3(1.0f), frontLeftTirePos, wheelOrientation, invMassWheel, gravity, wheelFriction);
        auto frontRightWheel = Truck::createAndAddWheel(WheelPosition::FrontRight, glmvec3(1.0f), frontRightTirePos, wheelOrientation, invMassWheel, gravity, wheelFriction);

		//----------------------------------------------
		// JOINTS CREATION
        
        // Hinge axis for wheel rotation (along the x-axis for normal wheel behavior)
        const glmvec3 hingeAxis{ 1.0_real, 0.0_real, 0.0_real };

        // Offsets for joint positions relative to tire positions
        // width of tires are approx 1.04 units
        glmvec3 rightJointOffset = glmvec3(0.52, 0.0_real, 0.0_real);
        glmvec3 leftJointOffset = glmvec3(-0.52, 0.0_real, 0.0_real);

		// Joints are positioned inward from tire centres (with half the tire width)
        glmvec3 frontRightJointPos = frontRightTirePos + rightJointOffset;
        glmvec3 frontLeftJointPos = frontLeftTirePos + leftJointOffset;
        glmvec3 rearRightJointPos = rearRightTirePos + rightJointOffset;
        glmvec3 rearLeftJointPos = rearLeftTirePos + leftJointOffset;

        // Create joints based on these tire positions (adjust as per your joint creation logic)
        auto rearLeft = std::make_shared<VPEWorld::HingeJoint>(frame, rearLeftWheel, rearLeftJointPos, hingeAxis);
        auto rearRight = std::make_shared<VPEWorld::HingeJoint>(frame, rearRightWheel, rearRightJointPos, hingeAxis);
        auto frontLeft = std::make_shared<VPEWorld::HingeJoint>(frame, frontLeftWheel, frontLeftJointPos, hingeAxis);
        auto frontRight = std::make_shared<VPEWorld::HingeJoint>(frame, frontRightWheel, frontRightJointPos, hingeAxis);

		// Add the joints to the physics world
        m_physics->addConstraint(rearLeft);
        m_physics->addConstraint(rearRight);
		m_physics->addConstraint(frontLeft);
		m_physics->addConstraint(frontRight);

    }
}
