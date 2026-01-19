#pragma once

#include "VEInclude.h"
#include "VPE.hpp"

using namespace vpe;

namespace ve {
    class TrialBike {
    public:
        struct BikeSpawnContext {
            glmvec3 camPos{};
            glmvec3 fwd{};
            glmvec3 fwdN{};
            glmvec3 centre{};
            glmvec3 rearPos{};
            glmvec3 frontPos{};
        };

        struct BikeParams {
            real wheelbase = 3.0_real;
            real axleHeight = 1.33_real;
            real invMassFrame = 1.0_real / 100.0_real;
            real invMassWheel = 1.0_real / 300.0_real;
            real wheelFriction = 5.0_real;

            glmvec3 hingeAxis{ 1.0_real, 0.0_real, 0.0_real };

        };

        enum class ObstacleTrackType {
            BumpyRoad = 0,     
            TallHill = 1,      
            SteppedDrops = 2   
        };

        // jump state 
        bool m_jumpActive = false;
        real m_jumpTimeRemaining = 0.5_real;
        real m_jumpTimer = 0.0_real;
        static constexpr uint64_t JUMP_FORCE_ID = 1000;
        static constexpr uint64_t PRE_JUMP_FORCE_ID = 2000;

        // bike state
        std::shared_ptr<VPEWorld::Body> m_frame{};
        std::shared_ptr<VPEWorld::Body> m_rearWheel{};
        std::shared_ptr<VPEWorld::Body> m_frontWheel{};
        std::shared_ptr<VPEWorld::HingeJoint> m_rearHinge{};
        BikeSpawnContext m_cachedCtx{};

        TrialBike(VPEWorld* physics, VPEWorld::callback_move onMove, VPEWorld::callback_erase onErase)
            : m_physics{ physics }, m_onMove{ onMove }, m_onErase{ onErase } {
        }

        ~TrialBike() = default;


        void assembleBike();
        void setEngine(bool on, int i);
        void jump();
        void applyFrontSuspensionSpring();
        glm::vec3 getFramePosition() const;
        void setObstacleTrackType(ObstacleTrackType t) { m_trackType = t; }
        void createObstacles(const BikeSpawnContext& ctx);

    private:

        
        static constexpr const char* bikeDir = "../../media/models/trial-bike";
        static constexpr const char* crateDir = "../../media/models/test/crate0";
        static constexpr uint32_t springForceA = 7878;
        static constexpr uint32_t springForceB = 7879;

        BikeParams makeBikeParams() const;
        BikeSpawnContext computeBikeSpawnContext(const BikeParams& p) const;

        std::shared_ptr<VPEWorld::Body> createAndAddObject(
            const std::string& modelDir,
            const std::string& modelFile,
            VPEWorld::Polytope* collider,
            glmvec3 scale,
            glmvec3 position,
            glmquat orientation,
            real inv_mass,
            bool gravity,
            real friction = 1.0_real
        );

        std::shared_ptr<VPEWorld::Body> createFrame(const BikeSpawnContext& ctx, const BikeParams& p);

        void createBiker(
            const BikeParams& p
        );

        void createWheels(
            const BikeSpawnContext& ctx,
            const BikeParams& p
        );

        std::shared_ptr<VPEWorld::HingeJoint> attachRearWheelHinge(
            const BikeSpawnContext& ctx,
            const BikeParams& p
        );

        std::shared_ptr<VPEWorld::Body> createPassiveHandles(
 
            const BikeSpawnContext& ctx
        );

        std::shared_ptr<VPEWorld::Body> createActiveHandlesAndSlider(
            const std::shared_ptr<VPEWorld::Body>& handlesPassive,
            const BikeSpawnContext& ctx,
            const BikeParams& p
        );

        
        

    private:
        VPEWorld* m_physics = nullptr;
        VPEWorld::callback_move m_onMove{};
        VPEWorld::callback_erase m_onErase{};
        
        //motor
        real m_targetOmega = 6.0_real;
        real m_maxMotorTorque = 80.0_real;

        // front suspension / handles
        std::shared_ptr<VPEWorld::SliderJoint> m_handleSlider{};
    public:
        std::shared_ptr<VPEWorld::Body> m_handles{};
        std::shared_ptr<VPEWorld::Body> m_handlesActive{};
    
    private:
        glmvec3 m_sliderLocalA{};
        glmvec3 m_sliderLocalB{};
        glmvec3 m_sliderAxisLocalA{};

        //obstacles
        ObstacleTrackType m_trackType = ObstacleTrackType::SteppedDrops;

    };

}


