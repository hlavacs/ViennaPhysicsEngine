#pragma once

#include "VEInclude.h"
#include "VPE.hpp"

using namespace vpe;

namespace ve {

    enum class WheelPosition {
        FrontLeft,
        FrontRight,
        RearRight,
        RearLeft
    };


    class Truck {
        VPEWorld* m_physics;
        VPEWorld::callback_move m_onMove;
        VPEWorld::callback_erase m_onErase;

    public:
        // Constructor and Destructor
        Truck(VPEWorld* physics, VPEWorld::callback_move onMove, VPEWorld::callback_erase onErase) : m_physics{ physics }, m_onMove{ onMove }, m_onErase{ onErase } {}
        ~Truck() {}

        /// <summary>
        /// Creates a truck body object, adds it to the physics world, and returns the pointer to the VPEWorld::Body created.
        /// </summary>
        /// <param name="scale">Scale of the body</param>
        /// <param name="position">Position of the body</param>
        /// <param name="orientation">Orientation of the body</param>
        /// <param name="inv_mass">Inverse mass (for physics calculations)</param>
        /// <param name="gravity">Whether to apply gravity</param>
        /// <param name="friction">Friction value</param>
        /// <returns>Returns a shared pointer to the VPEWorld::Body created</returns>
        std::shared_ptr<VPEWorld::Body> createAndAddBody(glmvec3 scale, glmvec3 position, glmquat orientation, real inv_mass, bool gravity, real friction = 1.0_real);

        /// <summary>
        /// Creates a wheel object, adds it to the physics world, and returns the pointer to the VPEWorld::Body created.
        /// </summary>
		/// <param name="wheelPosition">Position of the wheel (FrontLeft, FrontRight, RearLeft, RearRight)</param>
        /// <param name="scale">Scale of the wheel</param>
        /// <param name="position">Position of the wheel</param>
        /// <param name="orientation">Orientation of the wheel</param>
        /// <param name="inv_mass">Inverse mass (for physics calculations)</param>
        /// <param name="gravity">Whether to apply gravity</param>
        /// <param name="friction">Friction value</param>
        /// <returns>Returns a shared pointer to the VPEWorld::Body created</returns>
        std::shared_ptr<VPEWorld::Body> createAndAddWheel(WheelPosition wheelPosition, glmvec3 scale, glmvec3 position, glmquat orientation, real inv_mass, bool gravity, real friction = 1.0_real);
        
        void truckFourWheels();
    };


}