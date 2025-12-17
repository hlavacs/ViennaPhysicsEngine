#include "Truck.hpp"
#ifndef DEBUG_Truck
#define DEBUG_Truck 1
#endif

using namespace vpe;

namespace ve {

	//Static member variable definitions
	//My idea was to define all truck parameters here so they can be easily adjusted if needed
	//I chose static members so that they are shared across all Truck instances
	//Many of these parameters are adjustable in the runtime GUI as well, i wanted to keep the settings independent of the instance
	//So changing them in the GUI affects every truck instance created afterwards
    
    //Mass
    real Truck::s_massFrame = 2000.0_real;
    real Truck::s_massWheel = 50.0_real;
	
    //Wheel friction
    real Truck::s_wheelFriction = 1.5_real;

	//Spring settings
    real Truck::s_suspensionStiffness = 25000.0_real;
    real Truck::s_suspensionDamping = 2500.0_real;
    
	//Motor settings
    real Truck::s_motorSpeed = 15.0_real;
    real Truck::s_forceMotor = 30.0_real;

	//Steering settings
    real Truck::s_steerAngle = 15.0_real;


    std::shared_ptr<VPEWorld::Body> Truck::createAndAddFrame(glmvec3 scale, glmvec3 position, glmquat orientation, real inv_mass, bool gravity, real friction) {

        // Load the truck model
        VESceneNode* model;
        VECHECKPOINTER(model = getSceneManagerPointer()->loadModel("Model" + std::to_string(m_physics->m_body_id), "../../media/models/truck", "invisible_cube.obj", 0, getRoot()));

        //Create the truck body with the specified parameters
        auto body = std::make_shared<VPEWorld::Body>(m_physics, "Frame" + std::to_string(m_physics->m_bodies.size()), model, &m_physics->g_cube, scale, position, orientation, glmvec3{ 0.0_real }, glmvec3{ 0.0_real }, inv_mass, m_physics->m_restitution, friction);

        //Set callbacks for movement and erasure
        body->m_on_move = m_onMove;
        body->m_on_erase = m_onErase;

        //Add the body to the physics world
        m_physics->addBody(body);

        //Apply gravity if needed
        body->setForce(0ul, VPEWorld::Force{ {0, 0, 0} });
        if (gravity) {
            body->setForce(0ul, VPEWorld::Force{ {0 , m_physics->c_gravity, 0} });
        }
        return body;
    }

    std::shared_ptr<VPEWorld::Body> Truck::createAndAddChassis(glmvec3 scale, glmvec3 position, glmquat orientation, real inv_mass, bool gravity, real friction) {

        // Load the truck model
        VESceneNode* model;
        VECHECKPOINTER(model = getSceneManagerPointer()->loadModel("Model" + std::to_string(m_physics->m_body_id), "../../media/models/truck", "Truck.obj", 0, getRoot()));

        //Create the truck body with the specified parameters
        auto body = std::make_shared<VPEWorld::Body>(m_physics, "Chassis" + std::to_string(m_physics->m_bodies.size()), model, &m_physics->g_cube, scale, position, orientation, glmvec3{ 0.0_real }, glmvec3{ 0.0_real }, inv_mass, m_physics->m_restitution, friction);

        //Set callbacks for movement and erasure
        body->m_on_move = m_onMove;
        body->m_on_erase = m_onErase;

        //Add the body to the physics world
        m_physics->addBody(body);

        //Apply gravity if needed
        body->setForce(0ul, VPEWorld::Force{ {0, 0, 0} });
        if (gravity) {
            body->setForce(0ul, VPEWorld::Force{ {0 , m_physics->c_gravity, 0} });
        }
        return body;
    }

    std::shared_ptr<VPEWorld::Body> Truck::createAndAddWheel(glmvec3 scale, glmvec3 position, glmquat orientation, real inv_mass, bool gravity) {
        VESceneNode* model;

        //Load the wheel model
        VECHECKPOINTER(model = getSceneManagerPointer()->loadModel("Model" + std::to_string(m_physics->m_body_id), "../../media/models/truck", "TruckWheel.obj", 0, getRoot()));

        //Create the wheel body with the specified parameters
        auto wheel = std::make_shared<VPEWorld::Body>(m_physics, "Wheel" + std::to_string(m_physics->m_bodies.size()), model, &m_physics->g_cylinder, scale, position, orientation, glmvec3{ 0.0_real }, glmvec3{ 0.0_real }, inv_mass, m_physics->m_restitution, Truck::s_wheelFriction);

        //Set callbacks for movement and erasure
        wheel->m_on_move = m_onMove;
        wheel->m_on_erase = m_onErase;

        //Add the wheel to the physics world
        m_physics->addBody(wheel);

        //Apply gravity if needed
        wheel->setForce(0ul, VPEWorld::Force{ {0, 0, 0} });
        if (gravity) {
            wheel->setForce(0ul, VPEWorld::Force{ {0, m_physics->c_gravity, 0} });
        }

        return wheel;
    }

    void  Truck::truck() {
		//Get camera position and forward direction
        glmvec3 camPos{ getSceneManagerPointer()->getSceneNode("StandardCameraParent")->getWorldTransform()[3] };
        glmvec3 fwd{ getSceneManagerPointer()->getSceneNode("StandardCamera")->getWorldTransform()[2] };

        const bool gravity = true;

		const real invMassFrame = 1 / Truck::s_massFrame;       //For stability, the frame is much heavier than the chassis. Frame is positioned lower to the ground to lower the center of mass
		const real invMassChassis = invMassFrame * 100.0_real;  //Chassis is lighter to maintain low center of mass, and it provides a collision body above the frame

        glmvec3 truckFrameCenter = camPos + 30.0_real * fwd;
		truckFrameCenter.y = 10.0_real; //Set a fixed height for the truck frame, initially the truck falls onto the ground

		//Create Truck Frame
		glmvec3 truckFrameScale{ 2.5_real, 0.5_real, 5.0_real };    //Flat frame
        auto frame = Truck::createAndAddFrame(truckFrameScale, truckFrameCenter, glmquat{ 1,0,0,0 }, invMassFrame, gravity, 0.0_real);

        //Create Truck Chassis
		glmvec3 truckChassisScale{ 2.5_real, 3.0_real, 5.0_real };  //Tall chassis, positioned above the frame, visual object is scaled inversely, so after scaling it looks normal
		glmvec3 truckChassisCenter = truckFrameCenter + glmvec3(0.0_real, 1.75_real, 0.0_real); //Calculated position to sit perfectly above the frame
        auto chassis = Truck::createAndAddChassis(truckChassisScale, truckChassisCenter, glmquat{ 1,0,0,0 }, invMassChassis, gravity, 0.0_real);

		//Connect Frame and Chassis with two fixed joints for stability
		//Joint Nr 1
		glmvec3 truckAnchorPointFront = truckFrameCenter + glmvec3(1.0_real, 0.25_real, 0.0_real);
        auto truckFrameChassisJointFront = std::make_shared<VPEWorld::FixedJoint>(frame, chassis, truckAnchorPointFront);
		truckFrameChassisJointFront->setRotationBias(1.0_real);     //High bias to avoid jittering
		truckFrameChassisJointFront->setTranslationBias(1.0_real);  //High bias to avoid jittering
		m_physics->addConstraint(truckFrameChassisJointFront);

        //Joint Nr 2
        glmvec3 truckAnchorPointBack = truckFrameCenter + glmvec3(-1.0_real, 0.25_real, 0.0_real);
        auto truckFrameChassisJointBack = std::make_shared<VPEWorld::FixedJoint>(frame, chassis, truckAnchorPointBack);
        truckFrameChassisJointBack->setRotationBias(1.0_real);      //High bias to avoid jittering
        truckFrameChassisJointBack->setTranslationBias(1.0_real);   //High bias to avoid jittering
        m_physics->addConstraint(truckFrameChassisJointBack);

		//Wheels

		//Wheel parameters
		glmquat wheelOrientation = glmquat(glmvec3(0.0f, glm::radians(90.0f), 0.0f)); //Rotate wheels to align with the truck, visual object is also rotated accordingly
		glmvec3 wheelScale{ 2.0_real, 2.0_real, 2.0_real }; //Scale wheels, visual object is also descaled accordingly, so after scaling it looks normal
		glmvec3 axisSuspension(0, 1, 0);    //Y axis for suspension movement
		glmvec3 axisRolling(1, 0, 0);       //X axis for wheel rolling

		//Calculated wheel positions relative to the truck frame center
        glmvec3 frontRightOffset = glmvec3(-2.0_real, 0.0_real, -1.67f);
        glmvec3 frontLeftOffset = glmvec3(2.0_real, 0.0_real, -1.67f);
        glmvec3 rearRightOffset = glmvec3(-2.0_real, 0.0_real, 1.53f);
        glmvec3 rearLeftOffset = glmvec3(2.0_real, 0.0_real, 1.53f);

		//Wheel world positions
        glmvec3 frontRightTirePos = truckFrameCenter + frontRightOffset;
        glmvec3 frontLeftTirePos = truckFrameCenter + frontLeftOffset;
        glmvec3 rearRightTirePos = truckFrameCenter + rearRightOffset;
        glmvec3 rearLeftTirePos = truckFrameCenter + rearLeftOffset;

		const real invMassWheel = 1 / Truck::s_massWheel;
		
        //Create wheels
        auto frontRightWheel = Truck::createAndAddWheel(wheelScale, frontRightTirePos, wheelOrientation, invMassWheel, gravity);
        auto frontLeftWheel = Truck::createAndAddWheel(wheelScale, frontLeftTirePos, wheelOrientation, invMassWheel, gravity);
        auto rearRightWheel = Truck::createAndAddWheel(wheelScale, rearRightTirePos, wheelOrientation, invMassWheel, gravity);
        auto rearLeftWheel = Truck::createAndAddWheel(wheelScale, rearLeftTirePos, wheelOrientation, invMassWheel, gravity);

		//Offset for the suspension joints to position them correctly relative to the wheels. (wheels have some width)
        glmvec3 rightJointOffset = glmvec3(0.5, 0.0_real, 0.0_real);
        glmvec3 leftJointOffset = glmvec3(-0.5, 0.0_real, 0.0_real);

		//Real world positions of the suspension joint anchor points
        glmvec3 frAnchorPointWheel = frontRightTirePos + rightJointOffset;
        glmvec3 flAnchorPointWheel = frontLeftTirePos + leftJointOffset;
        glmvec3 rrAnchorPointWheel = rearRightTirePos + rightJointOffset;
        glmvec3 rlAnchorPointWheel = rearLeftTirePos + leftJointOffset;

		//Create suspension joints for each wheel

		//Wheel Front Right
        auto frJoint = std::make_shared<VPEWorld::AdvancedWheelSuspensionJoint>(frame, frontRightWheel, frAnchorPointWheel, axisRolling, axisSuspension);
        frJoint->setWheelSide(VPEWorld::AdvancedWheelSuspensionJoint::WheelSide::RIGHT);
        frJoint->setWheelPosition(VPEWorld::AdvancedWheelSuspensionJoint::WheelPosition::FRONT);
        frJoint->setSpringParameters(Truck::s_suspensionStiffness, Truck::s_suspensionDamping);
        m_physics->addConstraint(frJoint);

		//Wheel Front Left
        auto flJoint = std::make_shared<VPEWorld::AdvancedWheelSuspensionJoint>(frame, frontLeftWheel, flAnchorPointWheel, axisRolling, axisSuspension);
        flJoint->setWheelSide(VPEWorld::AdvancedWheelSuspensionJoint::WheelSide::LEFT);
        flJoint->setWheelPosition(VPEWorld::AdvancedWheelSuspensionJoint::WheelPosition::FRONT);
        flJoint->setSpringParameters(Truck::s_suspensionStiffness, Truck::s_suspensionDamping);
        m_physics->addConstraint(flJoint);

		//Wheel Rear Right
        auto rrJoint = std::make_shared<VPEWorld::AdvancedWheelSuspensionJoint>(frame, rearRightWheel, rrAnchorPointWheel, axisRolling, axisSuspension);
        rrJoint->setWheelSide(VPEWorld::AdvancedWheelSuspensionJoint::WheelSide::RIGHT);
        rrJoint->setWheelPosition(VPEWorld::AdvancedWheelSuspensionJoint::WheelPosition::REAR);
        rrJoint->setSpringParameters(Truck::s_suspensionStiffness, Truck::s_suspensionDamping);
        m_physics->addConstraint(rrJoint);

		//Wheel Rear Left
        auto rlJoint = std::make_shared<VPEWorld::AdvancedWheelSuspensionJoint>(frame, rearLeftWheel, rlAnchorPointWheel, axisRolling, axisSuspension);
        rlJoint->setWheelSide(VPEWorld::AdvancedWheelSuspensionJoint::WheelSide::LEFT);
        rlJoint->setWheelPosition(VPEWorld::AdvancedWheelSuspensionJoint::WheelPosition::REAR);
        rlJoint->setSpringParameters(Truck::s_suspensionStiffness, Truck::s_suspensionDamping);
        m_physics->addConstraint(rlJoint);     

    }

	//Camera follow logic
    void Truck::followTruckCamera(real dt) {
        auto sceneNode = getSceneManagerPointer()->getSceneNode("StandardCameraParent");
        if (!sceneNode) return;

        CameraMode currentMode = m_cameraMode;

        for (auto& body : m_physics->m_bodies) {
            if (body.second->m_name.find("Chassis") != std::string::npos) {

                glmvec3 truckPos = body.second->m_positionW;
                glmvec3 truckForward = glm::normalize(-glmvec3(body.second->m_model[2]));
                glmvec3 truckUp = glm::normalize(glmvec3(body.second->m_model[1]));

                real distBehind = 0.0_real;
                real heightOffset = 0.0_real;

                switch (currentMode) {
                case THIRD_PERSON_WIDE:
                    //Wide, high, traditional third-person view
                    distBehind = 30.0_real;
                    heightOffset = 12.0_real;
                    break;

                case THIRD_PERSON_CLOSE:
                    //Closer, lower, more dramatic view for jumps
                    distBehind = 15.0_real;
                    heightOffset = 5.0_real;
                    break;

                case DRIVER_VIEW:
                    //Inside the cabin, slightly offset up and back
                    distBehind = -2.5_real;
                    heightOffset = 3.5_real;
                    break;

                case FRONT_VIEW:
                    //Inside the cabin, slightly offset up and back
                    distBehind = -15.0_real;
                    heightOffset = 2.5_real;
                    break;

                default:
                    //Default to wide if something goes wrong
                    distBehind = 25.0_real;
                    heightOffset = 9.0_real;
                    break;
                }

                //Calculate Target Position
                glmvec3 targetCamPos = truckPos - (truckForward * distBehind) + glmvec3(0.0_real, heightOffset, 0.0_real);

                //For driver view, adjust the position to be relative to the truck's orientation
                if (currentMode == DRIVER_VIEW) {
                    //For driver view, use the truck's forward and up vectors directly
                    targetCamPos = truckPos
                        + (truckUp * 2.5_real)          // Offset up slightly based on truck angle
                        - (truckForward * 0.5_real);    // Small offset back
                }

                glmvec3 currentCamPos{ sceneNode->getWorldTransform()[3] };

				real lerpSpeed = 5.0_real;  //Speed of camera movement
                glmvec3 newPos = glm::mix(currentCamPos, targetCamPos, lerpSpeed * (real)dt);

                sceneNode->setPosition(newPos);
                sceneNode->lookAt(newPos,truckPos + glmvec3(0.0_real, 2.0_real, -5.0_real),glmvec3(0.0_real, 1.0_real, 0.0_real));

                break;
            }
        }
    }

	//Truck control methods
    void Truck::truckMoveForward() {
		//Iterate through all constraints to find the wheel joints
        for (auto& constraint : m_physics->m_constraints) {
            auto advancedSuspensionJoint = std::dynamic_pointer_cast<VPEWorld::AdvancedWheelSuspensionJoint>(constraint);
            if (advancedSuspensionJoint) {
				//Enable motor on wheel to move forward
                advancedSuspensionJoint->enableMotor(-Truck::s_motorSpeed, Truck::s_forceMotor);
                continue;
            }
        }
    }

    void Truck::truckMoveLeft() {
		//Iterate through all constraints to find the wheel joints
        for (auto& constraint : m_physics->m_constraints) {
            auto advancedSuspensionJoint = std::dynamic_pointer_cast<VPEWorld::AdvancedWheelSuspensionJoint>(constraint);
            if (advancedSuspensionJoint) {
				//Enable motor on wheel to move forward and steer left on front wheels
                if (advancedSuspensionJoint->getWheelPosition() == VPEWorld::AdvancedWheelSuspensionJoint::WheelPosition::FRONT) {
					advancedSuspensionJoint->enableSteering(-Truck::s_steerAngle * (pi / 180.0_real));  //Convert degrees to radians
                }
                advancedSuspensionJoint->enableMotor(-Truck::s_motorSpeed, Truck::s_forceMotor);
                continue;
            }
        }
    }

    void Truck::truckMoveRight() {
        //Iterate through all constraints to find the wheel joints
        for (auto& constraint : m_physics->m_constraints) {
            //Enable motor on wheel to move forward and steer right on front wheels
            auto advancedSuspensionJoint = std::dynamic_pointer_cast<VPEWorld::AdvancedWheelSuspensionJoint>(constraint);
            if (advancedSuspensionJoint) {
                if (advancedSuspensionJoint->getWheelPosition() == VPEWorld::AdvancedWheelSuspensionJoint::WheelPosition::FRONT) {
                    advancedSuspensionJoint->enableSteering(Truck::s_steerAngle * (pi / 180.0_real));
                }
                advancedSuspensionJoint->enableMotor(-Truck::s_motorSpeed, Truck::s_forceMotor);
                continue;
            }
        }
    }

    void Truck::truckMoveBackward() {
		//Iterate through all constraints to find the wheel joints
        for (auto& constraint : m_physics->m_constraints) {
            auto advancedSuspensionJoint = std::dynamic_pointer_cast<VPEWorld::AdvancedWheelSuspensionJoint>(constraint);
            if (advancedSuspensionJoint) {
				//Enable motor on wheel to move backward
                advancedSuspensionJoint->enableMotor(Truck::s_motorSpeed, Truck::s_forceMotor);
                continue;
            }
        }
    }

    void Truck::truckRoll() {
		//Iterate through all constraints to find the wheel joints
        for (auto& constraint : m_physics->m_constraints) {
            auto advancedSuspensionJoint = std::dynamic_pointer_cast<VPEWorld::AdvancedWheelSuspensionJoint>(constraint);
            if (advancedSuspensionJoint) {
				//Disable motor and steering to let the truck roll freely
				advancedSuspensionJoint->disableSteering();
                advancedSuspensionJoint->disableMotor();
            }
        }
    }

    void Truck::truckHandBrake() {
		//Iterate through all constraints to find the wheel joints
        for (auto& constraint : m_physics->m_constraints) {
            auto advancedSuspensionJoint = std::dynamic_pointer_cast<VPEWorld::AdvancedWheelSuspensionJoint>(constraint);
            if (advancedSuspensionJoint) {
				//Enable strong motor resistance on rear wheels to simulate handbrake
                if (advancedSuspensionJoint->getWheelPosition() == VPEWorld::AdvancedWheelSuspensionJoint::WheelPosition::REAR) {
                    advancedSuspensionJoint->enableMotor(0.0_real, 2000.0_real);
                }
            }
        }
    }



	//Damping getters and setters
    real Truck::getSuspensionDamping() {
        return Truck::s_suspensionDamping;
    }

    void Truck::increaseDamping() {
        Truck::s_suspensionDamping += 50.0_real;
        for (auto& constraint : m_physics->m_constraints) {
            auto advancedWheelSuspensionJoint = std::dynamic_pointer_cast<VPEWorld::AdvancedWheelSuspensionJoint>(constraint);
            if (advancedWheelSuspensionJoint) {
                advancedWheelSuspensionJoint->setSpringParameters(Truck::s_suspensionStiffness, Truck::s_suspensionDamping);
            }
        }
    }

    void Truck::decreaseDamping() {
        Truck::s_suspensionDamping -= 50.0_real;
        if (Truck::s_suspensionDamping < 10.0_real) {
            Truck::s_suspensionDamping = 10.0_real;
        }
        for (auto& constraint : m_physics->m_constraints) {
            auto advancedWheelSuspensionJoint = std::dynamic_pointer_cast<VPEWorld::AdvancedWheelSuspensionJoint>(constraint);
            if (advancedWheelSuspensionJoint) {
                advancedWheelSuspensionJoint->setSpringParameters(Truck::s_suspensionStiffness, Truck::s_suspensionDamping);
            }
        }
    }

	//Stiffness getters and setters
    real Truck::getSuspensionStiffness() {
        return Truck::s_suspensionStiffness;
    }

    void Truck::increaseStiffness() {
        Truck::s_suspensionStiffness += 500.0_real;
        for (auto& constraint : m_physics->m_constraints) {
            auto advancedWheelSuspensionJoint = std::dynamic_pointer_cast<VPEWorld::AdvancedWheelSuspensionJoint>(constraint);
            if (advancedWheelSuspensionJoint) {
                advancedWheelSuspensionJoint->setSpringParameters(Truck::s_suspensionStiffness, Truck::s_suspensionDamping);
            }
        }
    }
    
    void Truck::decreaseStiffness() {
        Truck::s_suspensionStiffness -= 500.0_real;
        if (Truck::s_suspensionStiffness < 50.0_real) {
            Truck::s_suspensionStiffness = 50.0_real;
        }
        for (auto& constraint : m_physics->m_constraints) {
            auto advancedWheelSuspensionJoint = std::dynamic_pointer_cast<VPEWorld::AdvancedWheelSuspensionJoint>(constraint);
            if (advancedWheelSuspensionJoint) {
                advancedWheelSuspensionJoint->setSpringParameters(Truck::s_suspensionStiffness, Truck::s_suspensionDamping);
            }
        }
    }

	//Friction getters and setters
    real Truck::getWheelFriction() {
        return Truck::s_wheelFriction;
	}

    void Truck::setFriction() {
        for (auto const& [id, body] : m_physics->m_bodies) {
            if (body->m_name.find("Wheel") != std::string::npos) {
                body->m_friction = Truck::s_wheelFriction;
            }
        }
    }

    void Truck::increaseWheelFriction(){
        if (Truck::s_wheelFriction < 1)
            Truck::s_wheelFriction += 0.1_real;
        else 
            Truck::s_wheelFriction += 1.0_real;

        if (Truck::s_wheelFriction > 15.0_real) {
            Truck::s_wheelFriction = 15.0_real;
        }
		Truck::setFriction();
	}

    void Truck::decreaseWheelFriction() {
        if (Truck::s_wheelFriction < 1)
            Truck::s_wheelFriction -= 0.1_real;
        else
            Truck::s_wheelFriction -= 1.0_real;

        if (Truck::s_wheelFriction < 0.1_real) {
            Truck::s_wheelFriction = 0.1_real;
        }
        Truck::setFriction();

    }

    //Truck Mass getter and setter
    real Truck::getMassFrame() {
        return Truck::s_massFrame;
	}

	void Truck::setMassFrame() {
        for (auto const& [id, body] : m_physics->m_bodies) {
            if (body->m_name.find("Frame") != std::string::npos) {
                body->m_mass_inv = 1 / Truck::s_massFrame;
                body->inertiaTensorL();
                body->updateMatrices();
            }
        }
    }

    void Truck::increaseMassFrame() {
		Truck::s_massFrame += 500.0_real;
        if (Truck::s_massFrame > 10000.0_real) {
            Truck::s_massFrame = 10000.0_real;
        }
        Truck::setMassFrame();
	}

    void Truck::decreaseMassFrame() {
        Truck::s_massFrame -= 500.0_real;
        if (Truck::s_massFrame < 1000.0_real) {
            Truck::s_massFrame = 1000.0_real;
        }
        Truck::setMassFrame();
    }

    //Wheel Mass getter and setter
    real Truck::getMassWheel() {
        return Truck::s_massWheel;
    }

    void Truck::setMassWheel() {
        for (auto const& [id, body] : m_physics->m_bodies) {
            if (body->m_name.find("Wheel") != std::string::npos) {
                body->m_mass_inv = 1 / Truck::s_massWheel;
                body->inertiaTensorL();
                body->updateMatrices();
            }
        }
    }

    void Truck::increaseMassWheel() {
        Truck::s_massWheel += 20.0_real;
        if (Truck::s_massWheel > 200.0_real) {
            Truck::s_massWheel = 200.0_real;
        }
        Truck::setMassWheel();
    }

    void Truck::decreaseMassWheel() {
        Truck::s_massWheel -= 20.0_real;
        if (Truck::s_massWheel < 20.0_real) {
            Truck::s_massWheel = 20.0_real;
        }
        Truck::setMassWheel();
    }

    //Motor Speed getter and setters
    real Truck::getMotorSpeed() {
        return Truck::s_motorSpeed;
    }
  
    void Truck::decreaseMotorSpeed() {
        Truck::s_motorSpeed -= 1.0_real;
        if (Truck::s_motorSpeed < 1.0_real) {
            Truck::s_motorSpeed = 1.0_real;
		}
	}

    void Truck::increaseMotorSpeed() {
        Truck::s_motorSpeed += 1.0_real;
        if (Truck::s_motorSpeed > 30.0_real) {
            Truck::s_motorSpeed = 30.0_real;
        }
    }

	//Motor Force getter and setters
    real Truck::getMotorForce() {
        return Truck::s_forceMotor;
    }
    
    void Truck::increaseMotorForce() {
        Truck::s_forceMotor += 5.0_real;
        if (Truck::s_forceMotor > 100.0_real) {
            Truck::s_forceMotor = 100.0_real;
        }
	}

    void Truck::decreaseMotorForce() {
        Truck::s_forceMotor -= 5.0_real;
        if (Truck::s_forceMotor < 5.0_real) {
            Truck::s_forceMotor = 5.0_real;
        }
	}

    //Steering Angle getter and setters
    real Truck::getSteerAngle() {
        return Truck::s_steerAngle;
    }
    
    void Truck::increaseSteerAngle() {
        Truck::s_steerAngle += 5.0_real;
        if (Truck::s_steerAngle > 20.0_real) {
            Truck::s_steerAngle = 20.0_real;
        }
    }

    void Truck::decreaseSteerAngle() {
        Truck::s_steerAngle -= 5.0_real;
        if (Truck::s_steerAngle < 0.0_real) {
            Truck::s_steerAngle = 0.0_real;
        }
    }

}
