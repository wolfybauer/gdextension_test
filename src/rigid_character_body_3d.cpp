#include "rigid_character_body_3d.hpp"
#include "godot_cpp/classes/animatable_body3d.hpp"
#include "godot_cpp/classes/object.hpp"
#include "godot_cpp/core/class_db.hpp"
#include "godot_cpp/core/math.hpp"
#include "godot_cpp/variant/vector3.hpp"
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/physics_server3d.hpp>

using namespace godot;

VARIANT_ENUM_CAST(RigidCharacterBody3D::PlatformOnExit);


void RigidCharacterBody3D::_detect_ceiling_floor_wall() {
    PhysicsServer3D * serv = PhysicsServer3D::get_singleton();
    if(!serv) {
        // TODO error + abort
    }
    _floor_normal = Vector3(0,0,0);
    _state = serv->body_get_direct_state(get_rid());
    if(!_state) {
        // TODO error + abort
    }

    _is_on_floor = false;
    _is_on_ceiling = false;
    _is_on_wall = false;

    AnimatableBody3D * moving_platform = nullptr;
    AnimatableBody3D * ab;

    for(int i=0; i<_state->get_contact_count(); i++) {
        Object * collider = _state->get_contact_collider_object(i);
        if(!collider) {
            // TODO error + abort
        }

        Vector3 contact_pos = to_local(_state->get_contact_collider_position(i));
        Vector3 normal = _state->get_contact_local_normal(i);
        float contact_angle_rad = Math::acos(normal.dot(up_direction));

        // check on wall
        if(
            contact_pos.y > floor_knee_height &&
            contact_pos.y < neck_height &&
            contact_angle_rad > floor_max_angle_rad &&
            contact_angle_rad < ceiling_min_angle_rad
        ) {
            _is_on_wall = true;
            continue;
        }

        // check on ceiling
        if(
            contact_pos.y >= neck_height &&
            (contact_angle_rad >= ceiling_min_angle_rad ||
            Math::is_equal_approx(contact_angle_rad, ceiling_min_angle_rad))
        ) {
            _is_on_ceiling= true;
            continue;
        }

        // check moving platform
        if(normal.y > _floor_normal.y) {
            _floor_normal = normal;
            ab = Object::cast_to<AnimatableBody3D>(collider);
            if(ab != nullptr) {
                moving_platform = ab;
            } else {
                moving_platform = nullptr;
            }
        }

        // check on floor, apply slide
        if(contact_angle_rad <= floor_max_angle_rad || Math::is_equal_approx(contact_angle_rad, floor_max_angle_rad)) {
            _is_on_floor = true;

            if(!floor_stop_on_slope) {
                continue;
            }

            Vector3 normal_force = -get_gravity().length() * get_mass() * Vector3(0,-1,0).slide(normal);
            apply_central_force(normal_force);
        }
    }

    if(!platform_move_with) {
        return;
    }

    // move body if on moving platform
    if(moving_platform != nullptr) {
        reparent(moving_platform);
    } else if(_is_on_floor && _initial_parent != nullptr) {
        reparent(_initial_parent);
    }

    Node * p = get_parent();
    ab = Object::cast_to<AnimatableBody3D>(p);
    if(ab == nullptr) {
        return;
    }

    Vector3 platform_vel = serv->body_get_state(ab->get_rid(), PhysicsServer3D::BODY_STATE_LINEAR_VELOCITY);
    Vector3 lin_vel = get_linear_velocity();
    if(!_is_on_floor && lin_vel.dot(platform_vel) <= 0.0f) {
        if(platform_on_exit == PLATFORM_ADD_VEL) {
            lin_vel += platform_vel;
            set_linear_velocity(lin_vel);
        } else if(platform_on_exit == PLATFORM_ADD_UP_VEL && platform_vel.y > 0.0f) {
            lin_vel.y += platform_vel.y;
            set_linear_velocity(lin_vel);
        }
    }
}

void RigidCharacterBody3D::move_and_slide() {
    _detect_ceiling_floor_wall();

    if(target_velocity.is_zero_approx()) {
        return;
    }

    float move_mag = get_mass() * acceleration_magnitude;
    Vector3 move_force = move_mag * target_velocity.normalized();
    float target_speed = target_velocity.length();

    move_force = modify_move_force(move_force);

    if(_is_on_floor) {
        if(_floor_normal.is_normalized() && move_force.normalized().dot(_floor_normal) > 0.1f) {
            move_force = move_force.slide(_floor_normal);
        }
        if(!floor_constant_speed && target_velocity.normalized().dot(_floor_normal) < -0.1f) {
            move_force *= Math::clamp<float>(_floor_normal.dot(up_direction), -1.0f, 1.0f);
            target_speed = target_velocity.slide(_floor_normal).length();
        }
    }

    Vector3 lin_vel = get_linear_velocity();
    Vector3 hor_vel = Vector3(lin_vel.x, 0.0f, lin_vel.z);
    float drag_scale = hor_vel.length() / target_speed;
    Vector3 drag_force = move_mag * drag_scale * -hor_vel.normalized();

    apply_force(move_force + drag_force);
}

Vector3 RigidCharacterBody3D::modify_move_force(Vector3 force) {
    if (has_method("_modify_move_force")) {
        Variant ret = call("_modify_move_force", force);
        if (ret.get_type() == Variant::VECTOR3) {
            return ret;
        }
    }
    return force;
}


void RigidCharacterBody3D::_bind_methods() {
	// methods
	ClassDB::bind_method(D_METHOD("move_and_slide"), &RigidCharacterBody3D::move_and_slide);
    // ClassDB::bind_method(D_METHOD("_modify_move_force", "force"), &RigidCharacterBody3D::modify_move_force);

	ClassDB::bind_method(D_METHOD("is_on_ceiling"), &RigidCharacterBody3D::is_on_ceiling);
	ClassDB::bind_method(D_METHOD("is_on_ceiling_only"), &RigidCharacterBody3D::is_on_ceiling_only);
	ClassDB::bind_method(D_METHOD("is_on_floor"), &RigidCharacterBody3D::is_on_floor);
	ClassDB::bind_method(D_METHOD("is_on_floor_only"), &RigidCharacterBody3D::is_on_floor_only);
	ClassDB::bind_method(D_METHOD("is_on_wall"), &RigidCharacterBody3D::is_on_wall);
	ClassDB::bind_method(D_METHOD("is_on_wall_only"), &RigidCharacterBody3D::is_on_wall_only);

	// properties
    ClassDB::bind_method(D_METHOD("set_target_velocity", "velocity"), &RigidCharacterBody3D::set_target_velocity);
	ClassDB::bind_method(D_METHOD("get_target_velocity"), &RigidCharacterBody3D::get_target_velocity);
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "target_velocity"), "set_target_velocity", "get_target_velocity");

    ClassDB::bind_method(D_METHOD("set_acceleration_magnitude", "height"), &RigidCharacterBody3D::set_acceleration_magnitude);
	ClassDB::bind_method(D_METHOD("get_acceleration_magnitude"), &RigidCharacterBody3D::get_acceleration_magnitude);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "acceleration_magnitude"), "set_acceleration_magnitude", "get_acceleration_magnitude");

	ClassDB::bind_method(D_METHOD("set_up_direction", "direction"), &RigidCharacterBody3D::set_up_direction);
	ClassDB::bind_method(D_METHOD("get_up_direction"), &RigidCharacterBody3D::get_up_direction);
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "up_direction"), "set_up_direction", "get_up_direction");

	ClassDB::bind_method(D_METHOD("set_neck_height", "height"), &RigidCharacterBody3D::set_neck_height);
	ClassDB::bind_method(D_METHOD("get_neck_height"), &RigidCharacterBody3D::get_neck_height);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "neck_height"), "set_neck_height", "get_neck_height");

	ClassDB::bind_method(D_METHOD("set_ceiling_min_angle_rad", "radians"), &RigidCharacterBody3D::set_ceiling_min_angle_rad);
	ClassDB::bind_method(D_METHOD("get_ceiling_min_angle_rad"), &RigidCharacterBody3D::get_ceiling_min_angle_rad);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "ceiling_min_angle", PROPERTY_HINT_RANGE, "0,180,1,radians_as_degrees"), "set_ceiling_min_angle_rad", "get_ceiling_min_angle_rad");

	ClassDB::bind_method(D_METHOD("set_floor_stop_on_slope", "enabled"), &RigidCharacterBody3D::set_floor_stop_on_slope);
	ClassDB::bind_method(D_METHOD("get_floor_stop_on_slope"), &RigidCharacterBody3D::get_floor_stop_on_slope);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "floor_stop_on_slope"), "set_floor_stop_on_slope", "get_floor_stop_on_slope");

	ClassDB::bind_method(D_METHOD("set_floor_constant_speed", "enabled"), &RigidCharacterBody3D::set_floor_constant_speed);
	ClassDB::bind_method(D_METHOD("get_floor_constant_speed"), &RigidCharacterBody3D::get_floor_constant_speed);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "floor_constant_speed"), "set_floor_constant_speed", "get_floor_constant_speed");

	ClassDB::bind_method(D_METHOD("set_floor_knee_height", "height"), &RigidCharacterBody3D::set_floor_knee_height);
	ClassDB::bind_method(D_METHOD("get_floor_knee_height"), &RigidCharacterBody3D::get_floor_knee_height);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "floor_knee_height"), "set_floor_knee_height", "get_floor_knee_height");

	ClassDB::bind_method(D_METHOD("set_floor_max_angle_rad", "radians"), &RigidCharacterBody3D::set_floor_max_angle_rad);
	ClassDB::bind_method(D_METHOD("get_floor_max_angle_rad"), &RigidCharacterBody3D::get_floor_max_angle_rad);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "floor_max_angle", PROPERTY_HINT_RANGE, "0,180,1,radians_as_degrees"), "set_floor_max_angle_rad", "get_floor_max_angle_rad");

	ClassDB::bind_method(D_METHOD("set_platform_move_with", "enabled"), &RigidCharacterBody3D::set_platform_move_with);
	ClassDB::bind_method(D_METHOD("get_platform_move_with"), &RigidCharacterBody3D::get_platform_move_with);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "platform_move_with"), "set_platform_move_with", "get_platform_move_with");

	ClassDB::bind_method(D_METHOD("set_platform_on_exit", "mode"), &RigidCharacterBody3D::set_platform_on_exit);
	ClassDB::bind_method(D_METHOD("get_platform_on_exit"), &RigidCharacterBody3D::get_platform_on_exit);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "platform_on_exit", PROPERTY_HINT_ENUM,
		"Add Velocity,Add Up Velocity,Do Nothing"),
		"set_platform_on_exit", "get_platform_on_exit");

	// enum
	BIND_ENUM_CONSTANT(PLATFORM_ADD_VEL);
	BIND_ENUM_CONSTANT(PLATFORM_ADD_UP_VEL);
	BIND_ENUM_CONSTANT(PLATFORM_DO_NOTHING);
}


void RigidCharacterBody3D::_on_ready() {
    if(Engine::get_singleton()->is_editor_hint()) {
        return;
    }

    // ok if is null
    _initial_parent = get_parent();

    _phys_mat.instantiate();
    _phys_mat->set_friction(0.0f);
    set_physics_material_override(_phys_mat);

    // so character doesn't tip over
    set_axis_lock(PhysicsServer3D::BODY_AXIS_ANGULAR_X, true);
    set_axis_lock(PhysicsServer3D::BODY_AXIS_ANGULAR_Y, true);
    set_axis_lock(PhysicsServer3D::BODY_AXIS_ANGULAR_Z, true);

    set_contact_monitor(true);
    set_max_contacts_reported(16);
}

void RigidCharacterBody3D::_notification(int p_what) {
    switch (p_what) {
		case NOTIFICATION_READY: {
            _on_ready();
            break;
        }
        default:
            break;
    }
}
