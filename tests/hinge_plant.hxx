///
/// @file hinge_plant.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief Physical test plant for the closed-loop tests: a torque-driven
/// rotational joint simulated internally with a rigid body physics engine.
/// The simulation backend is encapsulated here so the test bodies only see
/// the mechanical interface (angle, rate, torque, step).
/// @version 0.2
/// @date 2026-09-16
/// SPDX-License-Identifier: MIT
/// @copyright Copyright (c) 2025
///

#pragma once

#include <ode/ode.h>
#include <stdexcept>

namespace test_plant {

///
/// @brief Process-wide handle on the physics library, initialized on first
/// use and torn down at program exit.
///
/// The underlying library is a process-global singleton requiring one
/// init/close pair; this guard satisfies that for any number of plants
/// without exposing the library to callers.
///
class physics_runtime {
public:
    /// @brief Obtain the shared runtime, initializing the library on first use.
    /// @throw std::runtime_error If the physics library fails to initialize.
    static physics_runtime& instance() {
        static physics_runtime runtime;
        return runtime;
    }

    physics_runtime(const physics_runtime&) = delete;
    physics_runtime& operator=(const physics_runtime&) = delete;

private:
    physics_runtime() {
        if (dInitODE2(0) == 0) { throw std::runtime_error("physics library init failed"); }
    }

    ~physics_runtime() { dCloseODE(); }
};

///
/// @brief RAII owner of a single rotational degree of freedom: a rigid body
/// on a hinge joint, driven by a torque actuator, with optional constant
/// load torque and viscous damping.
///
class hinge_plant {
public:
    ///
    /// @param load_torque Constant torque (N*m) about the hinge axis applied
    ///                    every step, modelling a persistent load.
    /// @param damping Angular viscous damping scale applied to the body.
    ///
    explicit hinge_plant(const double load_torque = 0.0, const double damping = 0.01)
        : load_torque_(load_torque) {
        physics_runtime::instance(); // ensure the library is up first

        world_ = dWorldCreate();
        dWorldSetGravity(world_, 0.0, 0.0, 0.0); // shaft is horizontal

        body_ = dBodyCreate(world_);
        dMass mass;
        dMassSetBoxTotal(&mass, /*total_mass=*/1.0, /*lx=*/1.0, /*ly=*/0.1, /*lz=*/0.1);
        dBodySetMass(body_, &mass);
        dBodySetPosition(body_, 0.5, 0.0, 0.0);

        group_ = dJointGroupCreate(0);
        hinge_ = dJointCreateHinge(world_, group_);
        dJointAttach(hinge_, body_, 0); // body to the fixed world
        dJointSetHingeAnchor(hinge_, 0.0, 0.0, 0.0);
        dJointSetHingeAxis(hinge_, 0.0, 0.0, 1.0); // rotation about +z

        // Viscous friction so an un-actuated joint settles.
        dBodySetDamping(body_, /*linear_scale=*/0.0, /*angular_scale=*/damping);
    }

    hinge_plant(const hinge_plant&) = delete;
    hinge_plant& operator=(const hinge_plant&) = delete;

    ~hinge_plant() {
        dJointDestroy(hinge_);
        dJointGroupDestroy(group_);
        dBodyDestroy(body_);
        dWorldDestroy(world_);
    }

    ///
    /// @brief Apply a torque (N*m) about the hinge and advance one step.
    ///
    /// The torque accumulates into the body and is consumed (and cleared)
    /// by the world step, so the command must be re-applied every step.
    ///
    void step(const double torque, const double dt) {
        dBodyAddTorque(body_, 0.0, 0.0, static_cast<dReal>(torque + load_torque_));
        dWorldQuickStep(world_, static_cast<dReal>(dt));
    }

    /// @brief Hinge angle (rad); zero is the pose set at joint creation,
    /// reported in (-pi, pi].
    double angle() const { return dJointGetHingeAngle(hinge_); }

    /// @brief Hinge angular rate (rad/s).
    double rate() const { return dJointGetHingeAngleRate(hinge_); }

    /// @brief Rotational inertia about the hinge z-axis (kg*m^2).
    double inertia_zz() const {
        dMass mass;
        dBodyGetMass(body_, &mass);
        return mass.I[dM3E_ZZ]; // zz element of the flat row-major tensor
    }

private:
    dWorldID world_;
    dBodyID body_;
    dJointGroupID group_;
    dJointID hinge_;
    double load_torque_;
};

} // namespace test_plant
