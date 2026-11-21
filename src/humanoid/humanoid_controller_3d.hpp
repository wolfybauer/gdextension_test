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

class HumanoidController3D : public Node {
    GDCLASS(HumanoidController3D, Node)

public:
    enum WeaponStyle { UNARMED, KNIFE, PISTOL, LONG_GUN, BAT };

    void configure(CharacterBody3D *body, HumanoidRig3D *rig);
    void configure_weapon(Node3D *weapon, WeaponStyle style, float cooldown, bool automatic, float recoil, bool reversed);
    void set_input(const Vector2 &move, const Vector2 &aim, bool trigger_held);
    void step(double delta);
    bool attack();
    void reload();
    void jump();
    bool play_reaction(const StringName &clip);
    Vector3 get_attack_direction() const;

    void set_move_speed(float speed);
    float get_move_speed() const { return move_speed; }
    bool is_aiming() const { return aiming; }

protected:
    static void _bind_methods();

private:
    enum class Action { LOCOMOTION, MELEE, RELOAD, BAT_SWING, REACTION };

    CharacterBody3D *body = nullptr;
    HumanoidRig3D *rig = nullptr;
    ObjectID weapon_id;
    WeaponStyle style = UNARMED;
    WeaponStyle impact_style = UNARMED;
    Action action = Action::LOCOMOTION;
    StringName action_clip;
    Vector2 move;
    Vector2 aim;
    Vector2 aim_correction;
    Vector3 swing_direction;
    float move_speed = 5.0f;
    float cooldown = 0.0f;
    float cooldown_left = 0.0f;
    float recoil_strength = 0.0f;
    float recoil_left = 0.0f;
    bool aiming = false;
    bool trigger_held = false;
    bool automatic = false;
    bool reversed = false;
    bool impact_pending = false;

    void update_aim();
    void update_backpedal_correction(float delta);
    void update_animation(bool bat_swing);
    void update_impact(AnimationPlayer *torso);
    bool play_action(Action next, const StringName &clip, double blend);
    void cancel_action();
    void on_torso_finished(const StringName &clip);
    bool has_firearm() const;
    WeaponStyle active_style() const;
    Node3D *get_weapon() const;
};

} // namespace godot

VARIANT_ENUM_CAST(godot::HumanoidController3D::WeaponStyle)
