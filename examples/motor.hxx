///
/// @file motor.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief Brushed DC motor plant for the example programs. The rigid-body
/// dynamics are simulated internally with a physics engine, but that
/// dependency is fully encapsulated: users see only the dc_motor class and
/// its electrical/mechanical interface.
/// @version 0.2
/// @date 2026-09-16
/// SPDX-License-Identifier: MIT
/// @copyright Copyright (c) 2025
///

#pragma once

#include <ode/ode.h>
#include <stdexcept>

namespace detail {

///
/// @brief Process-wide handle on the physics library, initialized on first
/// use and torn down at program exit.
///
/// The underlying library is a process-global singleton requiring one
/// init/close pair; this reference-counted guard satisfies that for any
/// number of plants without exposing the library to callers.
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
/// @brief A single rotational degree of freedom: a rigid body on a hinge
/// joint, driven by a torque actuator. Internally a physics-engine object.
///
class hinge_rotor {
public:
    ///
    /// @param inertia Rotor + load inertia about the shaft (kg*m^2).
    /// @param bearing Angular viscous damping coefficient (N*m*s/rad).
    /// @param load_torque Constant disturbance torque on the shaft (N*m).
    ///
    hinge_rotor(const double inertia, const double bearing, const double load_torque)
        : load_torque_(load_torque) {
        physics_runtime::instance(); // ensure the library is up before creating objects

        world_ = dWorldCreate();
        dWorldSetGravity(world_, 0.0, 0.0, 0.0); // horizontal shaft: no gravity torque

        body_ = dBodyCreate(world_);
        dMass mass;
        dMassSetParameters(
            &mass,
            /*total_mass=*/1.0,
            /*cg=*/0.0,
            0.0,
            0.0,
            /*I11=*/inertia,
            /*I22=*/inertia,
            /*I33=*/inertia,
            /*I12=*/0.0,
            /*I13=*/0.0,
            /*I23=*/0.0
        );
        dBodySetMass(body_, &mass);

        group_ = dJointGroupCreate(0);
        hinge_ = dJointCreateHinge(world_, group_);
        dJointAttach(hinge_, body_, 0); // body to the fixed world
        dJointSetHingeAnchor(hinge_, 0.0, 0.0, 0.0);
        dJointSetHingeAxis(hinge_, 0.0, 0.0, 1.0); // rotation about +z
        dBodySetDamping(body_, /*linear_scale=*/0.0, /*angular_scale=*/bearing);
    }

    hinge_rotor(const hinge_rotor&) = delete;
    hinge_rotor& operator=(const hinge_rotor&) = delete;

    ~hinge_rotor() {
        dJointDestroy(hinge_);
        dJointGroupDestroy(group_);
        dBodyDestroy(body_);
        dWorldDestroy(world_);
    }

    /// @brief Apply a shaft torque plus the constant load and advance one step.
    void step(const double torque, const double dt) {
        // The torque accumulates into the body and is consumed (cleared) by
        // the world step, so it is re-applied every step.
        dBodyAddTorque(body_, 0.0, 0.0, static_cast<dReal>(torque + load_torque_));
        dWorldQuickStep(world_, static_cast<dReal>(dt));
    }

    /// @brief Shaft angle (rad), reported in (-pi, pi].
    double angle() const { return dJointGetHingeAngle(hinge_); }

    /// @brief Shaft angular rate (rad/s).
    double rate() const { return dJointGetHingeAngleRate(hinge_); }

    /// @brief Return the rotor to its rest pose at zero speed.
    void reset() {
        const dQuaternion identity = { 1.0, 0.0, 0.0, 0.0 };
        dBodySetQuaternion(body_, identity);
        dBodySetAngularVel(body_, 0.0, 0.0, 0.0);
    }

private:
    dWorldID world_;
    dBodyID body_;
    dJointGroupID group_;
    dJointID hinge_;
    double load_torque_;
};

} // namespace detail

///
/// @brief Brushed DC motor model: ODE rotor mechanics plus an explicit
/// armature electrical circuit.
///
/// Armature: L * di/dt = V - R*i - Ke*omega
/// Torque:   tau = Kt * i (applied to the rotor)
///
/// The rigid-body simulation backing the mechanical subsystem is entirely
/// internal; this class only exposes electrical and shaft quantities.
///
class dc_motor {
public:
    /// Motor parameters (SI units).
    struct params {
        double resistance;         // Armature resistance R (ohm)
        double inductance;         // Armature inductance L (H)
        double torque_constant;    // Torque constant Kt (N*m/A)
        double back_emf_constant;  // Back-EMF constant Ke (V*s/rad)
        double inertia;            // Rotor + load inertia J (kg*m^2)
        double friction;           // Viscous bearing b (N*m*s/rad)
        double supply_voltage_max; // Command saturation (V)
        double load_torque = 0.0;  // Constant shaft disturbance (N*m)
    };

    explicit dc_motor(const params& p)
        : p_(p)
        , rotor_(p.inertia, p.friction, p.load_torque) {}

    dc_motor(const dc_motor&) = delete;
    dc_motor& operator=(const dc_motor&) = delete;

    ///
    /// @brief Integrate the armature circuit and advance the rotor one step.
    /// @param voltage Applied armature voltage (clamped to +-supply_voltage_max).
    /// @param dt Sample interval (seconds).
    /// @return The new angular velocity (rad/s).
    ///
    double update(const double voltage, const double dt) {
        const double v = voltage > p_.supply_voltage_max  ? p_.supply_voltage_max
                       : voltage < -p_.supply_voltage_max ? -p_.supply_voltage_max
                                                          : voltage;
        // Semi-implicit Euler for the current, then one mechanics step driven
        // by the resulting torque.
        current_ += (v - p_.resistance * current_ - p_.back_emf_constant * rotor_.rate()) * dt
                  / p_.inductance;
        rotor_.step(p_.torque_constant * current_, dt);
        return rotor_.rate();
    }

    /// @brief Rotor angular velocity (rad/s).
    double speed() const { return rotor_.rate(); }

    /// @brief Shaft angle (rad).
    double angle() const { return rotor_.angle(); }

    /// @brief Armature current (A).
    double current() const { return current_; }

    /// @brief Stop the rotor and de-energize the armature.
    void reset() {
        rotor_.reset();
        current_ = 0.0;
    }

private:
    params p_;
    detail::hinge_rotor rotor_;
    double current_ = 0.0; // Armature current
};
