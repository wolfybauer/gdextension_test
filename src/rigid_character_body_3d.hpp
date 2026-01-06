#pragma once

// #include "godot_cpp/classes/wrapped.hpp"
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/ref.hpp>
#include "godot_cpp/classes/physics_direct_body_state3d.hpp"
#include "godot_cpp/classes/physics_material.hpp"
#include "godot_cpp/core/math.hpp"
#include "godot_cpp/variant/vector3.hpp"
#include <godot_cpp/classes/rigid_body3d.hpp>

namespace godot {

class RigidCharacterBody3D : public RigidBody3D {
    GDCLASS(RigidCharacterBody3D, RigidBody3D);
    
public:
    enum PlatformOnExit {
        PLATFORM_ADD_VEL,
        PLATFORM_ADD_UP_VEL,
        PLATFORM_DO_NOTHING
    };

    RigidCharacterBody3D() = default;
    ~RigidCharacterBody3D() override = default;
    void _notification(int p_what);

    void move_and_slide();
    virtual Vector3 modify_move_force(Vector3 force);

    bool is_on_ceiling() { return _is_on_ceiling; }
    bool is_on_ceiling_only() { return _is_on_ceiling && !(_is_on_floor || _is_on_wall); }
    bool is_on_floor() { return _is_on_floor; }
    bool is_on_floor_only() { return _is_on_floor && !(_is_on_ceiling || _is_on_wall); }
    bool is_on_wall() { return _is_on_wall; }
    bool is_on_wall_only() { return _is_on_wall && !(_is_on_ceiling || _is_on_floor); }

    void set_up_direction(Vector3 d) { up_direction = d; }
    Vector3 get_up_direction() const { return up_direction; }
    void set_neck_height(float h) { neck_height = h; }
    float get_neck_height() const { return neck_height; }
    void set_ceiling_min_angle_rad(float r) { ceiling_min_angle_rad = r; }
    float get_ceiling_min_angle_rad() const { return ceiling_min_angle_rad; }
    
    void set_floor_stop_on_slope(bool s) { floor_stop_on_slope = s; }
    bool get_floor_stop_on_slope() const { return floor_stop_on_slope; }
    void set_floor_constant_speed(bool s) { floor_constant_speed = s; }
    bool get_floor_constant_speed() const { return floor_constant_speed; }
    void set_floor_knee_height(float h) { floor_knee_height = h; }
    float get_floor_knee_height() const { return floor_knee_height; }
    void set_floor_max_angle_rad(float r) { floor_max_angle_rad = r; }
    float get_floor_max_angle_rad() const { return floor_max_angle_rad; }

    void set_platform_move_with(bool m) { platform_move_with = m; }
    bool get_platform_move_with() const { return platform_move_with; }
    void set_platform_on_exit(int m) { if(m > 2 || m < 0) return; platform_on_exit = (enum PlatformOnExit)m; }
    int get_platform_on_exit() const { return (int)platform_on_exit; }

    void set_target_velocity(Vector3 t) { target_velocity = t; }
    Vector3 get_target_velocity() const { return target_velocity; }
    void set_acceleration_magnitude(float h) { acceleration_magnitude = h; }
    float get_acceleration_magnitude() const { return acceleration_magnitude; }

protected:
    static void _bind_methods();

private:

    // exports
    Vector3 up_direction = Vector3(0,1,0);
    float neck_height = 1.5f;
    float ceiling_min_angle_rad = Math::deg_to_rad(105.0f);

    bool floor_stop_on_slope = true;
    bool floor_constant_speed = true;
    float floor_knee_height = 0.5f;
    float floor_max_angle_rad = Math::deg_to_rad(45.0f);

    bool platform_move_with = true;
    enum PlatformOnExit platform_on_exit = PLATFORM_ADD_VEL;

    
    Vector3 target_velocity = Vector3(0,0,0);
    float acceleration_magnitude = 15.0f;

    // internals
    Node * _initial_parent = nullptr;
    PhysicsDirectBodyState3D * _state = nullptr;
    Ref<PhysicsMaterial> _phys_mat;
    Vector3 _floor_normal = Vector3(0,0,0);
    bool _is_on_ceiling = false;
    bool _is_on_floor = false;
    bool _is_on_wall = false;

    // funcs
    void _on_ready();
    void _detect_ceiling_floor_wall();
};
} // namespace godot
