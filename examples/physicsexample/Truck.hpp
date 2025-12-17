#pragma once

#include "VEInclude.h"
#include "VPE.hpp"

using namespace vpe;

namespace ve {

    ///<summary>
    ///Enumeration for available camera perspectives following the truck.
    ///</summary>
    enum CameraMode {
        THIRD_PERSON_WIDE = 0,      ///< Wide view behind the truck.
        THIRD_PERSON_CLOSE = 1,     ///< Close-up view behind the truck.
        DRIVER_VIEW = 2,            ///< View from the driver's perspective inside the cabin.
		FRONT_VIEW = 3,             ///< View from the front of the truck.
        NUM_CAMERA_MODES
    };


    ///<summary>
    ///Represents a truck with various physics and control capabilities
    ///within the VPE (Vienna Physics Engine) simulation.
    ///</summary>
    class Truck {
    private:
        VPEWorld* m_physics;
        VPEWorld::callback_move m_onMove;
        VPEWorld::callback_erase m_onErase;
        CameraMode m_cameraMode = THIRD_PERSON_WIDE;

    public:
		//Truck Body Parts masses
        static real s_massFrame;
        static real s_massWheel;

        //Suspension Parameters
        static real s_suspensionStiffness;
        static real s_suspensionDamping;

        //Control/Motor Parameters
        static real s_motorSpeed;
        static real s_forceMotor;
        static real s_steerAngle;
        static real s_forceSteering;

        //Wheel parameter
        static real s_wheelFriction;

        ///<summary>
        ///Constructor for the Truck class.
        ///</summary>
        ///<param name="physics">Pointer to the active VPEWorld instance for physics simulation.</param>
        ///<param name="onMove">Callback function to be executed when a body moves (e.g., for graphics update).</param>
        ///<param name="onErase">Callback function to be executed when a body is erased (e.g., for graphics cleanup).</param>
        Truck(VPEWorld* physics, VPEWorld::callback_move onMove, VPEWorld::callback_erase onErase) : m_physics{ physics }, m_onMove{ onMove }, m_onErase{ onErase } {}

        ///<summary>
        ///Destructor for the Truck class.
        ///</summary>
        ~Truck() {}

        ///<summary>
        ///Creates the main truck Frame object, adds it to the physics world, and returns the pointer.
        ///</summary>
        ///<param name="scale">Scale of the body.</param>
        ///<param name="position">Position of the body.</param>
        ///<param name="orientation">Orientation of the body (as a quaternion).</param>
        ///<param name="inv_mass">Inverse mass (for physics calculations).</param>
        ///<param name="gravity">Whether to apply gravity.</param>
        ///<param name="friction">Friction value (default is 1.0).</param>
        ///<returns>Returns a shared pointer to the VPEWorld::Body created.</returns>
        std::shared_ptr<VPEWorld::Body> createAndAddFrame(glmvec3 scale, glmvec3 position, glmquat orientation, real inv_mass, bool gravity, real friction = 1.0_real);

        ///<summary>
        ///Creates the main truck Chassis object, adds it to the physics world, and returns the pointer.
        ///</summary>
        ///<param name="scale">Scale of the body.</param>
        ///<param name="position">Position of the body.</param>
        ///<param name="orientation">Orientation of the body (as a quaternion).</param>
        ///<param name="inv_mass">Inverse mass (for physics calculations).</param>
        ///<param name="gravity">Whether to apply gravity.</param>
        ///<param name="friction">Friction value (default is 1.0).</param>
        ///<returns>Returns a shared pointer to the VPEWorld::Body created.</returns>
        std::shared_ptr<VPEWorld::Body> createAndAddChassis(glmvec3 scale, glmvec3 position, glmquat orientation, real inv_mass, bool gravity, real friction);

        ///<summary>
        ///Creates a wheel object, adds it to the physics world, and returns the pointer.
        ///</summary>
        ///<param name="scale">Scale of the wheel.</param>
        ///<param name="position">Position of the wheel.</param>
        ///<param name="orientation">Orientation of the wheel (as a quaternion).</param>
        ///<param name="inv_mass">Inverse mass (for physics calculations).</param>
        ///<param name="gravity">Whether to apply gravity.</param>
        ///<returns>Returns a shared pointer to the VPEWorld::Body created.</returns>
        std::shared_ptr<VPEWorld::Body> createAndAddWheel(glmvec3 scale, glmvec3 position, glmquat orientation, real inv_mass, bool gravity);


        ///<summary>
        ///Initializes the truck model using complex constraints for integrated suspension and steering logic, 
        ///enabling real-life movement simulation.
        ///</summary>
        void truck();

        //--- Camera & Update Methods ---

        ///<summary>
        ///Sets the camera position and orientation every frame based on the current m_cameraMode.
        ///</summary>
        ///<param name="dt">Delta time, the time elapsed since the last frame, used for frame-rate independent updates.</param>
        void followTruckCamera(real dt);

        ///<summary>
        ///Cycles to the next available camera mode: THIRD_PERSON_WIDE -> THIRD_PERSON_CLOSE -> DRIVER_VIEW -> FRONT_VIEW.
        ///</summary>
        void toggleCameraMode() {
            m_cameraMode = static_cast<CameraMode>((m_cameraMode + 1) % NUM_CAMERA_MODES);
        }

        //--- Truck Control Methods ---

        ///<summary>
        ///Applies the necessary forces/torques to accelerate the truck forward.
        ///</summary>
        void truckMoveForward();

        ///<summary>
        ///Applies steering input to turn the front wheels left for the truck.
        ///</summary>
        void truckMoveLeft();

        ///<summary>
        ///Applies steering input to turn the front wheels right for the truck.
        ///</summary>
        void truckMoveRight();

        ///<summary>
        ///Applies the necessary forces/torques to accelerate the truck backward or brake.
        ///</summary>
        void truckMoveBackward();

        ///<summary>
        ///Disables every active controls, returning the truck to a rolling state.
        ///</summary>
        void truckRoll();

        ///<summary>
        ///Stops the application of driving forces and applies braking to bring the truck to a halt.
        ///</summary>
        void truckHandBrake();


        //--- Suspension Adjustment Methods ---

        ///<summary>
        ///Increases the stiffness parameter of the truck's suspension springs.
        ///</summary>
        void increaseStiffness();

        ///<summary>
        ///Decreases the stiffness parameter of the truck's suspension springs.
        ///</summary>
        void decreaseStiffness();

        ///<summary>
        ///Increases the damping parameter of the truck's suspension shock absorbers.
        ///</summary>
        void increaseDamping();

        ///<summary>
        ///Decreases the damping parameter of the truck's suspension shock absorbers.
        ///</summary>
        void decreaseDamping();

        ///<summary>
        ///Returns the current stiffness value of the suspension springs.
        ///</summary>
        real getSuspensionStiffness();

        ///<summary>
        ///Returns the current damping value of the suspension shock absorbers.
        ///</summary>
        real getSuspensionDamping();


        //--- Wheel Friction Methods ---

        ///<summary>
        ///Returns the current friction coefficient of the wheels.
        ///</summary>
        real getWheelFriction();

        ///<summary>
        ///Directly sets the friction coefficient for the truck's wheels.
        ///</summary>
        void setFriction();

        ///<summary>
        ///Increments the friction coefficient to provide more traction.
        ///</summary>
        void increaseWheelFriction();

        ///<summary>
        ///Decrements the friction coefficient to allow for more sliding.
        ///</summary>
        void decreaseWheelFriction();


        //--- Mass and Physics Body Methods ---

        ///<summary>
        ///Returns the current mass of the main chassis/frame.
        ///</summary>
        real getMassFrame();

        ///<summary>
        ///Sets the mass of the truck's main frame.
        ///</summary>
        void setMassFrame();

        ///<summary>
        ///Increases the weight of the truck's main frame.
        ///</summary>
        void increaseMassFrame();

        ///<summary>
        ///Decreases the weight of the truck's main frame.
        ///</summary>
        void decreaseMassFrame();

        ///<summary>
        ///Returns the current mass of an individual wheel.
        ///</summary>
        real getMassWheel();

        ///<summary>
        ///Sets the mass of the truck's wheels.
        ///</summary>
        void setMassWheel();

        ///<summary>
        ///Increases the rotational inertia and weight of the wheels.
        ///</summary>
        void increaseMassWheel();

        ///<summary>
        ///Decreases the rotational inertia and weight of the wheels.
        ///</summary>
        void decreaseMassWheel();


        //--- Drivetrain and Motor Methods ---

        ///<summary>
        ///Returns the current target rotational speed of the motor.
        ///</summary>
        real getMotorSpeed();

        ///<summary>
        ///Decreases the target rotational speed of the motor.
        ///</summary>
        void decreaseMotorSpeed();

        ///<summary>
        ///Increases the target rotational speed of the motor.
        ///</summary>
        void increaseMotorSpeed();

        ///<summary>
        ///Returns the current maximum torque of the motor.
        ///</summary>
        real getMotorForce();

        ///<summary>
        ///Increases the torque applied by the motor to the wheels.
        ///</summary>
        void increaseMotorForce();

        ///<summary>
        ///Decreases the torque applied by the motor to the wheels.
        ///</summary>
        void decreaseMotorForce();


        //--- Steering Methods ---

        ///<summary>
        ///Returns the current maximum steering angle in degrees.
        ///</summary>
        real getSteerAngle();

        ///<summary>
        ///Increases the maximum turning radius/angle allowed for the wheels.
        ///</summary>
        void increaseSteerAngle();

        ///<summary>
        ///Decreases the maximum turning radius/angle allowed for the wheels.
        ///</summary>
        void decreaseSteerAngle();

    };

}