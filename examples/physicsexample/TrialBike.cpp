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
        const std::shared_ptr<VPEWorld::Body>& frame,
        const BikeParams& p,
        std::shared_ptr<VPEWorld::Body>& outBottom,
        std::shared_ptr<VPEWorld::Body>& outTop)
    {
        // slightly above the bike frame
        glmvec3 bikerPos = frame->m_positionW + glmvec3{ 0.0_real, 1.0_real, -0.4_real };
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
            frame,
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

        outBottom = biker_bottom;
        outTop = biker_top;
    }

    void TrialBike::createWheels(
        const BikeSpawnContext& ctx,
        const BikeParams& p,
        std::shared_ptr<VPEWorld::Body>& outRearWheel,
        std::shared_ptr<VPEWorld::Body>& outFrontWheel)
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

        outRearWheel = rearWheel;
        outFrontWheel = frontWheel;
    }

    std::shared_ptr<VPEWorld::HingeJoint> TrialBike::attachRearWheelHinge(
        const std::shared_ptr<VPEWorld::Body>& frame,
        const std::shared_ptr<VPEWorld::Body>& rearWheel,
        const BikeSpawnContext& ctx,
        const BikeParams& p)
    {
        auto rearHinge = std::make_shared<VPEWorld::HingeJoint>(
            frame,
            rearWheel,
            ctx.rearPos,
            p.hingeAxis
        );

        m_rearHinge = rearHinge;               
        rearHinge->setBody1MotorEnabled(false); 

        m_physics->addConstraint(rearHinge);
        return rearHinge;
    }

    std::shared_ptr<VPEWorld::Body> TrialBike::createPassiveHandles(
        const std::shared_ptr<VPEWorld::Body>& frame,
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
            0.1_real,
            false,
            0.5_real
        );

        m_handles = handles;

        // fixed joint to frame
        auto handleFixed = std::make_shared<VPEWorld::FixedJoint>(frame, handles, handlePos);
        m_physics->addConstraint(handleFixed);

        return handles;
    }

    std::shared_ptr<VPEWorld::Body> TrialBike::createActiveHandlesAndSlider(
        const std::shared_ptr<VPEWorld::Body>& handlesPassive,
        const std::shared_ptr<VPEWorld::Body>& frontWheel,
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
            0.15_real,
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
            frontWheel,
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

        // layout
        const int  numCubes = 6;
        const real spacing = 1.2_real;
        const real startDist = 6.0_real;
        const real cubeSize = 1.0_real;

        const glmquat cubeOri{ 1,0,0,0 };

        // first cube 
        glmvec3 firstCubePos = ctx.centre + startDist * fwdN;
        firstCubePos.y = 0.5_real * cubeSize;

        // ramp placement 
        const real rampScale = 1.0_real;
        const real halfCubeFwd = 0.5_real * cubeSize;
        const real halfRampFwd = 0.25_real * rampScale;

        glmvec3 rampPos = firstCubePos - (halfCubeFwd + halfRampFwd) * fwdN; 
        rampPos.y = 0.25_real * rampScale;                                  

        const glmquat rampOri = glm::angleAxis(glm::radians(180.0_real), up);

        createAndAddObject(
            bikeDir,
            "ramp.obj",
            &m_physics->g_ramp,
            glmvec3{ rampScale },
            rampPos,
            rampOri,
            1.0_real / 1000000.0_real,
            false,
            1.2_real
        );

        // cubes
        for (int i = 0; i < numCubes; ++i)
        {
            glmvec3 pos = ctx.centre + (startDist + i * spacing) * fwdN;
            pos.y = 0.5_real * cubeSize;

            createAndAddObject(
                crateDir,
                "cube.obj",
                &m_physics->g_cube,
                glmvec3{ cubeSize },
                pos,
                cubeOri,
                1.0_real / 1000000.0_real,
                false,
                1.0_real
            );
        }
    }

    // MAIN PIPELINE
    void TrialBike::assembleBike()
    {
        const BikeParams p = makeBikeParams();
        const BikeSpawnContext ctx = computeBikeSpawnContext(p);


        // create frame
        auto frame = createFrame(ctx, p);

        // create biker
        std::shared_ptr<VPEWorld::Body> biker_bottom;
        std::shared_ptr<VPEWorld::Body> biker_top;
        createBiker(frame, p, biker_bottom, biker_top);


        // wheels
        std::shared_ptr<VPEWorld::Body> rearWheel;
        std::shared_ptr<VPEWorld::Body> frontWheel;
        createWheels(ctx, p, rearWheel, frontWheel);

        // rear hinge driving
        auto rearHinge = attachRearWheelHinge(frame, rearWheel, ctx, p);

        // handles passive & fixed to frame
        auto handles = createPassiveHandles(frame, ctx);

        // active handles, slider, front wheel spin hinge
        auto handlesActive = createActiveHandlesAndSlider(handles, frontWheel, ctx, p);

        createObstacles(ctx);

    }

    void TrialBike::setEngine(bool on)
    {
        if (!on) {
            m_rearHinge->disableMotor();
            return;
        }

        m_rearHinge->enableMotor(m_targetOmega, m_maxMotorTorque);
        m_rearHinge->setBody1MotorEnabled(false);
    }

    void TrialBike::jump() {
        if (!m_frame) return;

        // upward force
        VPEWorld::Force jumpForce;
        jumpForce.m_forceW = glmvec3{ 0.0_real, 5000.0_real, 0.0_real };
        m_frame->setForce(JUMP_FORCE_ID, jumpForce);

        // timer activation
        m_jumpActive = true;
    }

    void TrialBike::applyFrontSuspensionSpring()
    {
        if (!m_handleSlider || !m_handles || !m_handlesActive) return;

        // axis and anchor points in world space
        const glmvec3 axisW = glm::normalize(m_handles->m_orientationLW * m_sliderAxisLocalA);

        const glm::vec3 pA = m_handles->m_positionW + (m_handles->m_orientationLW * m_sliderLocalA);
        const glm::vec3 pB = m_handlesActive->m_positionW + (m_handlesActive->m_orientationLW * m_sliderLocalB);

        // displacement along axis
        const real x = glm::dot(pB - pA, axisW);

        // point velocities along axis
        const glm::vec3 rA = pA - m_handles->m_positionW;
        const glm::vec3 rB = pB - m_handlesActive->m_positionW;

        const glm::vec3 vA = m_handles->m_linear_velocityW + glm::cross(m_handles->m_angular_velocityW, rA);
        const glm::vec3 vB = m_handlesActive->m_linear_velocityW + glm::cross(m_handlesActive->m_angular_velocityW, rB);

        const real v = glm::dot(vB - vA, axisW);

        // spring
        const real x0 = -0.2_real;
        const real k = 6000.0_real;
        const real d = 500.0_real;

        real F = -k * (x - x0) - d * v;

        // hard stops
        const real minX = -0.2_real;
        const real maxX = 0.10_real;
        const real stopK = 50000.0_real;
        const real stopD = 2000.0_real;

        if (x < minX) {
            const real penetration = x - minX;                
            const real vIntoStop = glm::min(v, 0.0_real);   
            F += -stopK * penetration - stopD * vIntoStop;
        }

        if (x > maxX) {
            const real penetration = x - maxX;                 
            const real vIntoStop = glm::max(v, 0.0_real);    
            F += -stopK * penetration - stopD * vIntoStop;
        }

        // applying equal and opposite forces
        m_handles->setForce(springForceA, VPEWorld::Force{ -F * axisW });
        m_handlesActive->setForce(springForceB, VPEWorld::Force{ F * axisW });
    }

    glm::vec3 TrialBike::getFramePosition() const
    {
        return m_frame ? m_frame->m_positionW : glm::vec3{ 0.0_real };
    }

}

