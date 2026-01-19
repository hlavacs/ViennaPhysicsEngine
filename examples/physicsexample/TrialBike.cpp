#include "TrialBike.hpp"
#ifndef DEBUG_BIKE
#define DEBUG_BIKE 1
#endif

using namespace vpe;

namespace ve {

    TrialBike::BikeParams TrialBike::makeBikeParams() const {
        return BikeParams{};
    }

    TrialBike::BikeSpawnContext TrialBike::computeBikeSpawnContext(const BikeParams& p) const {
        BikeSpawnContext ctx{};

        ctx.camPos = glmvec3{ getSceneManagerPointer()
            ->getSceneNode("StandardCameraParent")->getWorldTransform()[3] };

        ctx.fwd = glmvec3{ getSceneManagerPointer()
            ->getSceneNode("StandardCamera")->getWorldTransform()[2] };

        ctx.fwdN = glm::normalize(ctx.fwd);

        // centre of the bike, between axles 
        ctx.centre = ctx.camPos + 2.0_real * ctx.fwdN;
        ctx.centre[1] = p.axleHeight;

        // rear and front axle positions along the forward vector 
        ctx.rearPos = ctx.centre - 0.3_real * p.wheelbase * ctx.fwdN;
        ctx.frontPos = ctx.centre + 0.37_real * p.wheelbase * ctx.fwdN;

        ctx.centre[1] = 2.0_real;

        return ctx;
    }

    std::shared_ptr<VPEWorld::Body> TrialBike::createAndAddObject(
        const std::string& modelDir,
        const std::string& modelFile,
        VPEWorld::Polytope* collider,
        glmvec3 scale,
        glmvec3 position,
        glmquat orientation,
        real inv_mass,
        bool gravity,
        real friction
    ) {
        VESceneNode* model;
        VECHECKPOINTER(model = getSceneManagerPointer()->loadModel(
            "Model" + std::to_string(m_physics->m_body_id),
            modelDir,
            modelFile,
            0,
            getRoot()
        ));

        auto body = std::make_shared<VPEWorld::Body>(
            m_physics,
            "Body" + std::to_string(m_physics->m_bodies.size()),
            model,
            collider,
            scale,
            position,
            orientation,
            glmvec3{ 0.0_real },
            glmvec3{ 0.0_real },
            inv_mass,
            m_physics->m_restitution,
            friction
        );

        body->m_on_move = m_onMove;
        body->m_on_erase = m_onErase;

        m_physics->addBody(body);

        if (gravity) {
            body->setForce(0ul, VPEWorld::Force{ {0, m_physics->c_gravity, 0} });
        }

        return body;
    }

    std::shared_ptr<VPEWorld::Body> TrialBike::createFrame(const BikeSpawnContext& ctx, const BikeParams& p)
    {
        auto frame = createAndAddObject(
            bikeDir,
            "bike_body.obj",
            &m_physics->g_bike_body,
            glmvec3{ 1.0_real },
            ctx.centre,
            glmquat{ 1,0,0,0 },
            p.invMassFrame,
            true,
            0.6_real
        );

        m_frame = frame;
        frame->setAngularFactor(glmvec3{ 1.0_real, 0.0_real, 0.0_real });

        return frame;
    }

    void TrialBike::createBiker(
        const BikeParams& p
    )
    {
        // slightly above the bike frame
        glmvec3 bikerPos = m_frame->m_positionW + glmvec3{ 0.0_real, 1.0_real, -0.4_real };
        glmquat bikerOri{ 1, 0, 0, 0 };

        // biker bottom
        auto biker_bottom = createAndAddObject(
            bikeDir,
            "biker_bottom.obj",
            &m_physics->g_biker_bottom,
            glmvec3{ 1.0_real },
            bikerPos,
            bikerOri,
            1.0_real / 100.0_real,
            true,
            p.wheelFriction
        );

        // attach biker to the frame
        auto bikerJoint = std::make_shared<VPEWorld::FixedJoint>(
            m_frame,
            biker_bottom,
            bikerPos
        );
        m_physics->addConstraint(bikerJoint);

        // biker top
        auto biker_top = createAndAddObject(
            bikeDir,
            "biker_top.obj",
            &m_physics->g_biker_top,
            glmvec3{ 1.0_real },
            bikerPos,
            bikerOri,
            1.0_real / 100.0_real,
            true,
            p.wheelFriction
        );

        // hip pivot 
        glmvec3 hipAnchorW =
            bikerPos +
            glmvec3{ 0.0_real, -0.2_real, -0.1_real };

        glmvec3 hingeAxisW =
            glm::normalize(biker_bottom->m_orientationLW * glmvec3{ 1, 0, 0 });

        auto hipJoint = std::make_shared<VPEWorld::HingeJoint>(
            biker_bottom,
            biker_top,
            hipAnchorW,
            hingeAxisW
        );

        hipJoint->enableLimit(-0.261799_real, 0.0_real);
        m_physics->addConstraint(hipJoint);
    }

    void TrialBike::createWheels(
        const BikeSpawnContext& ctx,
        const BikeParams& p)
    {
        glmquat wheelOri{ 1,0,0,0 };

        auto rearWheel = createAndAddObject(
            bikeDir,
            "tire.obj",
            &m_physics->g_tire,
            glmvec3{ 1.0_real },
            ctx.rearPos,
            wheelOri,
            p.invMassWheel,
            true,
            p.wheelFriction
        );

        auto frontWheel = createAndAddObject(
            bikeDir,
            "tire.obj",
            &m_physics->g_tire,
            glmvec3{ 1.0_real },
            ctx.frontPos,
            wheelOri,
            p.invMassWheel,
            true,
            p.wheelFriction
        );

        rearWheel->setAngularFactor(glmvec3{ 1.0_real, 0.0_real, 0.0_real });
        frontWheel->setAngularFactor(glmvec3{ 1.0_real, 0.0_real, 0.0_real });

		m_rearWheel = rearWheel;
        m_frontWheel = frontWheel;
    }

    std::shared_ptr<VPEWorld::HingeJoint> TrialBike::attachRearWheelHinge(
        const BikeSpawnContext& ctx,
        const BikeParams& p)
    {
        auto rearHinge = std::make_shared<VPEWorld::HingeJoint>(
            m_frame,
            m_rearWheel,
            ctx.rearPos,
            p.hingeAxis
        );

        m_rearHinge = rearHinge;               
        rearHinge->setBody1MotorEnabled(false); 

        m_physics->addConstraint(rearHinge);
        return rearHinge;
    }

    std::shared_ptr<VPEWorld::Body> TrialBike::createPassiveHandles(
        const BikeSpawnContext& ctx)
    {
        // handle positions
        glmvec3 handlePos = ctx.centre + glmvec3{ 0.0_real, 0.4_real, 0.75_real };
        glmquat handleOri{ 1, 0, 0, 0 };

        // create passive handles
        auto handles = createAndAddObject(
            bikeDir,
            "handles_passive2.obj",
            &m_physics->g_handles_passive,
            glmvec3{ 1.0_real },
            handlePos,
            handleOri,
            1.0_real / 100.0_real,
            false,
            0.5_real
        );

        m_handles = handles;

        // fixed joint to frame
        auto handleFixed = std::make_shared<VPEWorld::FixedJoint>(m_frame, handles, handlePos);
        m_physics->addConstraint(handleFixed);

        return handles;
    }

    std::shared_ptr<VPEWorld::Body> TrialBike::createActiveHandlesAndSlider(
        const std::shared_ptr<VPEWorld::Body>& handlesPassive,
        const BikeSpawnContext& ctx,
        const BikeParams& p
    )
    {
        glmvec3 handlePos = ctx.centre + glmvec3{ 0.0_real, 0.4_real, 0.75_real };
        glmquat handleOri{ 1, 0, 0, 0 };

        // sliding handles
        glmvec3 activeStartPos = handlePos - glmvec3{ 0.0_real, 0.7_real, -0.26_real };

        auto handlesActive = createAndAddObject(
            bikeDir,
            "handles_active.obj",
            &m_physics->g_handles_active,
            glmvec3{ 1.0_real },
            activeStartPos,
            handleOri,
            1.0_real / 100.0_real,
            true,
            0.5_real
        );

        m_handlesActive = handlesActive;


        const glmvec3 sliderAxis{ 0.0_real, 1.0_real, -0.3_real };
        glmvec3 sliderAnchor = activeStartPos;

        auto handleSlider = std::make_shared<VPEWorld::SliderJoint>(
            handlesPassive,
            handlesActive,
            sliderAnchor,
            sliderAxis
        );

        m_handleSlider = handleSlider;


        m_sliderLocalA = glm::inverse(handlesPassive->m_orientationLW) * (sliderAnchor - handlesPassive->m_positionW);
        m_sliderLocalB = glm::inverse(handlesActive->m_orientationLW) * (sliderAnchor - handlesActive->m_positionW);
        m_sliderAxisLocalA = glm::normalize(glm::inverse(handlesPassive->m_orientationLW) * sliderAxis);

        m_physics->addConstraint(handleSlider);

        auto frontSpinHinge = std::make_shared<VPEWorld::HingeJoint>(
            handlesActive,
            m_frontWheel,
            ctx.frontPos,
            p.hingeAxis
        );
        m_physics->addConstraint(frontSpinHinge);

        return handlesActive;
    }

    void TrialBike::createObstacles(const BikeSpawnContext& ctx)
    {
        const glmvec3 fwdN = ctx.fwdN;
        const glmvec3 up{ 0.0_real, 1.0_real, 0.0_real };
        const glmvec3 rightN = glm::normalize(glm::cross(up, fwdN));

        const glmquat cubeOri{ 1,0,0,0 };

        const real cubeSize = 1.0_real;
        const real rampScale = 1.0_real;

        auto spawnCube = [&](glmvec3 posW, real size = 1.0_real, real baseY = 0.0_real)
            {
                posW.y = baseY + 0.5_real;

                createAndAddObject(
                    crateDir,
                    "cube.obj",
                    &m_physics->g_cube,
                    glmvec3{ size },
                    posW,
                    cubeOri,
                    1.0_real / 100000.0_real,
                    false,
                    1.0_real
                );
            };

        auto spawnRamp = [&](glmvec3 posW, glmquat oriW, real scale = 1.0_real, real friction = 1.2_real, real baseY = 0.0_real)
            {
                posW.y = baseY + 0.5_real;

                createAndAddObject(
                    bikeDir,
                    "ramp.obj",
                    &m_physics->g_ramp,
                    glmvec3{ scale },
                    posW,
                    oriW,
                    1.0_real / 100000.0_real,
                    false,
                    friction
                );
            };


        auto rampDownOri = [&]() { return glm::quat{ 1,0,0,0 }; };
        auto rampUpOri = [&]() { return glm::angleAxis(glm::radians(180.0_real), up); };

        // start in front of the bike
        const real startDist = 6.0_real;
        glmvec3 start = ctx.centre + startDist * fwdN;
        start.y = 0.0_real;

        switch (m_trackType)
        {
        case ObstacleTrackType::BumpyRoad:
        {
            const int segments = 10;

            real cursor = 0.0_real;

            for (int i = 0; i < segments; ++i)
            {
                // ramp up
                glmvec3 rampPosA = start + cursor * fwdN;
                spawnRamp(rampPosA, rampUpOri(), 1.0_real);
                cursor += cubeSize;

                // cube
                glmvec3 cubePos = start + cursor * fwdN;
                spawnCube(cubePos, cubeSize);
                cursor += cubeSize;

                // ramp down
                glmvec3 rampPosC = start + cursor * fwdN;
                spawnRamp(rampPosC, rampDownOri(), 1.0_real);
                cursor += cubeSize + 1.6_real;
            }
            break;
        }

        case ObstacleTrackType::TallHill: {
            auto spawnCube = [&](glmvec3 posW, real baseY)
                {
                    posW.y = baseY + 0.5_real; 

                    createAndAddObject(
                        crateDir, "cube.obj", &m_physics->g_cube,
                        glmvec3{ 1.0_real }, posW, cubeOri,
                        1.0_real / 100000.0_real, false, 1.0_real
                    );
                };

            auto spawnRamp = [&](glmvec3 posW, glmquat oriW, real friction, real baseY)
                {
                    posW.y = baseY + 0.5_real; 

                    createAndAddObject(
                        bikeDir, "ramp.obj", &m_physics->g_ramp,
                        glmvec3{ 1.0_real }, posW, oriW,
                        1.0_real / 100000.0_real, false, friction
                    );
                };
            real cursor = 0.0_real;
            std::vector<real> hillHeights = { 2.0_real, 3.0_real};

            for (size_t hillIndex = 0; hillIndex < hillHeights.size(); ++hillIndex) {
                int maxHeight = static_cast<int>(hillHeights[hillIndex]);

                // CLIMBING PHASE
                for (int h = 0; h < maxHeight; ++h) {
                    glm::vec3 p = start + cursor * fwdN;

                    // stack cubes 
                    for (int i = 0; i < h; ++i) {
                        spawnCube(p, (real)i);
                    }
                    // ramp on the top
                    spawnRamp(p, rampUpOri(), 1.2_real, (real)h);

                    cursor += cubeSize;
                }

                // THE PEAK 
                {
                    glm::vec3 p = start + cursor * fwdN;
                    for (int i = 0; i < maxHeight; ++i) {
                        spawnCube(p, (real)i);
                    }
                    cursor += cubeSize;
                }

                // DESCENT PHASE 
                for (int h = maxHeight - 1; h >= 0; --h) {
                    glm::vec3 p = start + cursor * fwdN;

                    // stack cubes 
                    for (int i = 0; i < h; ++i) {
                        spawnCube(p, (real)i);
                    }
                    // ramp on the top
                    spawnRamp(p, rampDownOri(), 1.2_real, (real)h);

                    cursor += cubeSize;
                }

                cursor += 4.0_real;
            }
            break;
        }

        case ObstacleTrackType::SteppedDrops:
        {
            auto spawnCube = [&](glmvec3 posW, real baseY) {
                posW.y = baseY + 0.5_real;
                createAndAddObject(
                    crateDir, "cube.obj", &m_physics->g_cube,
                    glmvec3{ 1.0_real }, posW, cubeOri,
                    1.0_real / 100000.0_real, false, 1.0_real
                );
                };

            auto spawnRamp = [&](glmvec3 posW, glmquat oriW, real friction, real baseY) {
                posW.y = baseY + 0.5_real;
                createAndAddObject(
                    bikeDir, "ramp.obj", &m_physics->g_ramp,
                    glmvec3{ 1.0_real }, posW, oriW,
                    1.0_real / 100000.0_real, false, friction
                );
                };

            real cursor = 0.0_real;
            std::vector<real> dropHeights = { 1.0_real, 2.0_real, 3.0_real };

            for (size_t i = 0; i < dropHeights.size(); ++i) {
                int maxHeight = static_cast<int>(dropHeights[i]);

                // CLIMBING PHASE 
                for (int h = 0; h < maxHeight; ++h) {
                    glm::vec3 p = start + cursor * fwdN;
                    for (int level = 0; level < h; ++level) {
                        spawnCube(p, (real)level);
                    }
                    spawnRamp(p, rampUpOri(), 1.2_real, (real)h);
                    cursor += cubeSize;
                }
                cursor += 6.0_real;
            }
            for (size_t i = 0; i < dropHeights.size(); ++i) {
                int maxHeight = static_cast<int>(dropHeights[i]);

                // CLIMBING PHASE 
                for (int h = 0; h < maxHeight; ++h) {
                    glm::vec3 p = start + cursor * fwdN;
                    for (int level = 0; level < h; ++level) {
                        spawnCube(p, (real)level);
                    }
                    spawnRamp(p, rampUpOri(), 1.2_real, (real)h);
                    cursor += cubeSize;
                }
                cursor += 6.0_real;
            }
            break;
        }

        default:
            break;
        }
    }

    // MAIN PIPELINE
    void TrialBike::assembleBike()
    {
        const BikeParams p = makeBikeParams();
        m_cachedCtx = computeBikeSpawnContext(p);

        // create frame
        auto frame = createFrame(m_cachedCtx, p);

        // wheels
        createWheels(m_cachedCtx, p);

        // rear hinge driving
        auto rearHinge = attachRearWheelHinge(m_cachedCtx, p);

        // handles passive & fixed to frame
        auto handles = createPassiveHandles(m_cachedCtx);

        // active handles, slider, front wheel spin hinge
        auto handlesActive = createActiveHandlesAndSlider(handles, m_cachedCtx, p);

        // create biker
        createBiker( p);
    }

    void TrialBike::setEngine(bool on,int i)
    {
        if (!on) {
            m_rearHinge->disableMotor();
            return;
        }

        m_rearHinge->enableMotor(i*m_targetOmega, m_maxMotorTorque);
        m_rearHinge->setBody1MotorEnabled(false);
    }

    void TrialBike::jump()
    {
        if (!m_frame || !m_rearWheel || !m_frontWheel)
            return;

        m_rearWheel->setForce(
            JUMP_FORCE_ID + 0,
            VPEWorld::Force{ { 0.0_real, 15.0_real, 0.0_real } }
        );

        m_frontWheel->setForce(
            JUMP_FORCE_ID + 1,
            VPEWorld::Force{ { 0.0_real, 35.0_real, 0.0_real } }
        );

        m_jumpActive = true;
        m_jumpTimer = 0.0_real;
        m_jumpTimeRemaining = 0.2_real;
    }


    void TrialBike::applyFrontSuspensionSpring()
    {
        if (!m_handleSlider || !m_handles || !m_handlesActive) return;

        // axis and anchor points in world space
        const glmvec3 axisW = glm::normalize(m_handles->m_orientationLW * m_sliderAxisLocalA);

        const glm::vec3 pA = m_handles->m_positionW + (m_handles->m_orientationLW * m_sliderLocalA);
        const glm::vec3 pB = m_handlesActive->m_positionW + (m_handlesActive->m_orientationLW * m_sliderLocalB);

        // displacement along axis, how far the suspension is compressed/extended
        const real x = glm::dot(pB - pA, axisW);

        // point velocities along axis
        const glm::vec3 rA = pA - m_handles->m_positionW;
        const glm::vec3 rB = pB - m_handlesActive->m_positionW;

        const glm::vec3 vA = m_handles->m_linear_velocityW + glm::cross(m_handles->m_angular_velocityW, rA);
        const glm::vec3 vB = m_handlesActive->m_linear_velocityW + glm::cross(m_handlesActive->m_angular_velocityW, rB);

        const real v = glm::dot(vB - vA, axisW);

        // spring
        const real x0 = -0.2_real;
        const real k = 500.0_real;
        const real d = 50.0_real;

        real F = -k * (x - x0) - d * v;

        // applying equal and opposite forces
        m_handles->setForce(springForceA, VPEWorld::Force{ -F * axisW });
        m_handlesActive->setForce(springForceB, VPEWorld::Force{ F * axisW });
    }

    glm::vec3 TrialBike::getFramePosition() const
    {
        return m_frame ? m_frame->m_positionW : glm::vec3{ 0.0_real };
    }



}

