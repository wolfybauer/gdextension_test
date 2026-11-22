#pragma once

#include "godot_cpp/classes/node.hpp"
#include "godot_cpp/core/object_id.hpp"
#include "godot_cpp/variant/vector2.hpp"
#include "godot_cpp/variant/vector3.hpp"

namespace godot {

class AnimationPlayer;
class CharacterBody3D;
class HumanoidRig3D;
class Node3D;
class PhysicsBody3D;
class RigidCharacterBody3D;

class HumanoidController3D : public Node {
    GDCLASS(HumanoidController3D, Node)

public:
    enum WeaponStyle { UNARMED, KNIFE, PISTOL, LONG_GUN, BAT };

    void configure(PhysicsBody3D *body, HumanoidRig3D *rig);
    void configure_weapon(Node3D *weapon, WeaponStyle style, float cooldown, bool automatic, float recoil, bool reversed, float spread_degrees = 0.0f);
    void set_input(const Vector2 &move, const Vector2 &aim, bool trigger_held);
    void step(double delta, bool animate = true);
    void set_aim_error(float degrees);
    bool react_to_hit(const Vector3 &direction, float knockback_speed, float recovery_seconds);
    void apply_knockback(const Vector3 &direction, float speed);
    bool attack();
    bool throw_item(Node3D *item);
    bool interact(const StringName &clip, const Vector3 &direction);
    bool is_smashing() const { return action == Action::MELEE && impact_style == LONG_GUN; }
    bool is_throwing() const { return action == Action::THROW; }
    void reload();
    void jump();
    bool play_reaction(const StringName &clip);
    double play_defeat();
    Vector3 get_attack_direction() const;

    void set_fall_animation_delay(float seconds);
    float get_fall_animation_delay() const { return fall_animation_delay; }
    void set_aim_move_speed_multiplier(float multiplier);
    float get_aim_move_speed_multiplier() const { return aim_move_speed_multiplier; }
    void set_move_speed(float speed);
    float get_move_speed() const { return move_speed; }
    bool is_aiming() const { return aiming; }
    bool is_weapon_equipped() const { return weapon_equipped; }

protected:
    static void _bind_methods();

private:
    enum class Action { LOCOMOTION, MELEE, RELOAD, BAT_SWING, REACTION, THROW, INTERACT };

    PhysicsBody3D *body = nullptr;
    CharacterBody3D *character_body = nullptr;
    RigidCharacterBody3D *rigid_body = nullptr;
    HumanoidRig3D *rig = nullptr;
    ObjectID weapon_id;
    ObjectID thrown_item_id;
    WeaponStyle style = UNARMED;
    WeaponStyle impact_style = UNARMED;
    Action action = Action::LOCOMOTION;
    StringName action_clip;
    Vector2 move;
    Vector2 aim;
    Vector2 aim_correction;
    Vector3 swing_direction;
    Vector3 knockback_velocity;
    float rigid_knockback_left = 0.0f;
    float move_speed = 5.0f;
    float aim_move_speed_multiplier = 0.75f;
    float cooldown = 0.0f;
    float cooldown_left = 0.0f;
    float recoil_strength = 0.0f;
    float recoil_left = 0.0f;
    float recoil_yaw = 0.0f;
    float spread_degrees = 0.0f;
    float aim_error_degrees = 0.0f;
    float hit_recovery_left = 0.0f;
    float airborne_time = 0.0f;
    float jump_start_grace_left = 0.0f;
    float fall_animation_delay = 0.10f;
    bool jumped = false;
    bool aiming = false;
    bool weapon_equipped = false;
    bool trigger_held = false;
    bool automatic = false;
    bool reversed = false;
    bool impact_pending = false;
    float interaction_time = 0.0f;
    float interaction_event_time = 0.0f;
    float interaction_duration = 0.0f;

    void move_body(const Vector2 &movement, float weight, float delta);
    bool is_on_floor() const;
    void update_aim();
    void update_backpedal_correction(float delta);
    void update_animation();
    void update_impact(AnimationPlayer *torso);
    void update_interaction(float delta);
    void update_weapon_mount();
    bool play_action(Action next, const StringName &clip, double blend, double speed = 1.0);
    void cancel_action();
    void on_torso_finished(const StringName &clip);
    bool has_firearm() const;
    WeaponStyle active_style() const;
    Node3D *get_weapon() const;
};

} // namespace godot

VARIANT_ENUM_CAST(godot::HumanoidController3D::WeaponStyle)
