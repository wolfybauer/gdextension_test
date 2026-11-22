#include "humanoid/humanoid_controller_3d.hpp"
#include "humanoid/humanoid_rig_3d.hpp"

#include "godot_cpp/classes/animation_player.hpp"
#include "godot_cpp/classes/character_body3d.hpp"
#include "godot_cpp/core/class_db.hpp"
#include "godot_cpp/core/math.hpp"
#include "godot_cpp/variant/utility_functions.hpp"

namespace godot {
namespace {

constexpr float RECOIL_DURATION = 0.15f;
constexpr float HIT_REACTION_SPEED = 1.75f;
constexpr float THROW_RELEASE_TIME = 0.80f;
constexpr float THROW_SPEED = 1.65f;
constexpr float AIM_DISTANCE = 3.0f;
constexpr float BAT_IMPACT_TIME = 0.32f / 1.2f;

struct AimProfile {
    const char *stance;
    bool use_chest;
    float normal_yaw;
    float normal_height;
    float reversed_yaw;
    float reversed_height;
};

constexpr AimProfile AIM_PROFILES[] = {
    {"unarmed", true, 22.5f, 1.2f, 22.5f, 1.2f},
    {"melee", true, 25.0f, 1.2f, 25.0f, 1.2f},
    {"pistol", true, 38.5f, 0.9f, 38.5f, 1.65f},
    {"twohand", true, 40.1f, 0.8f, 42.6f, 1.22f},
    {"bat", false, 0.0f, 0.0f, 0.0f, 0.0f},
};

StringName qualified(const StringName &clip) {
    return String("main/") + String(clip);
}

void play_loop(AnimationPlayer *player, const StringName &clip) {
    const StringName name = qualified(clip);
    if (player->get_current_animation() != name && player->has_animation(name)) {
        player->play(name, 0.15);
    }
}

float facing_yaw(const Vector3 &facing) {
    return Math::atan2(-facing.x, -facing.z);
}

} // namespace

void HumanoidController3D::configure(CharacterBody3D *new_body, HumanoidRig3D *new_rig) {
    const Callable finished = callable_mp(this, &HumanoidController3D::on_torso_finished);
    if (rig && rig->is_connected("torso_animation_finished", finished)) {
        rig->disconnect("torso_animation_finished", finished);
    }
    body = new_body;
    rig = new_rig;
    knockback_velocity = Vector3();
    hit_recovery_left = 0.0f;
    action = Action::LOCOMOTION;
    impact_pending = false;
    if (rig) {
        rig->connect("torso_animation_finished", finished);
    }
}

void HumanoidController3D::configure_weapon(Node3D *weapon, WeaponStyle new_style, float new_cooldown,
        bool new_automatic, float recoil, bool new_reversed, float spread) {
    if (new_style < UNARMED || new_style > BAT) {
        return;
    }
    if (action != Action::INTERACT) {
        cancel_action();
    }
    weapon_id = weapon ? ObjectID(weapon->get_instance_id()) : ObjectID();
    style = new_style;
    cooldown = MAX(0.0f, new_cooldown);
    automatic = new_automatic;
    recoil_strength = MAX(0.0f, recoil);
    spread_degrees = MAX(0.0f, spread);
    reversed = new_reversed;
    recoil_left = 0.0f;
    aim_correction = Vector2();
    update_weapon_mount();
}

void HumanoidController3D::set_input(const Vector2 &new_move, const Vector2 &new_aim, bool held) {
    move = new_move.limit_length();
    aim = new_aim.limit_length();
    trigger_held = held;
    const bool next_aiming = !aim.is_zero_approx();
    if (next_aiming == aiming) {
        return;
    }
    aiming = next_aiming;
    update_weapon_mount();
    if (!aiming) {
        if (action == Action::BAT_SWING || action == Action::RELOAD) {
            cancel_action();
        }
        recoil_left = 0.0f;
    }
    emit_signal("aiming_changed", aiming);
}

void HumanoidController3D::step(double seconds, bool animate) {
    if (!body || !rig || !rig->get_torso_player() || !rig->get_legs_player() || seconds <= 0.0) {
        return;
    }
    const float delta = seconds;
    hit_recovery_left = MAX(hit_recovery_left - delta, 0.0f);
    cooldown_left = MAX(cooldown_left - delta, 0.0f);
    if (animate && trigger_held && automatic && has_firearm()) {
        attack();
    }
    if (animate) {
        update_impact(rig->get_torso_player());
        update_interaction(delta);
    }
    const bool bat_swing = action == Action::BAT_SWING;
    const bool locked = bat_swing || action == Action::THROW || action == Action::INTERACT;
    const Vector2 movement = animate && !locked ? move : Vector2();
    const bool committed_swing = locked || action == Action::MELEE;
    const Vector2 facing = committed_swing ? Vector2(swing_direction.x, swing_direction.z) :
            (aiming ? aim : movement);
    const float weight = 1.0f - Math::pow(0.85f, delta * 60.0f);

    Vector3 velocity = body->get_velocity();
    velocity.x = Math::lerp(velocity.x, movement.x * move_speed / 1.5f, weight);
    velocity.z = Math::lerp(velocity.z, movement.y * move_speed, weight);
    if (!knockback_velocity.is_zero_approx()) {
        velocity.x = knockback_velocity.x;
        velocity.z = knockback_velocity.z;
    }
    if (!body->is_on_floor()) {
        velocity += body->get_gravity() * delta;
    }
    body->set_velocity(velocity);
    body->move_and_slide();
    if (!knockback_velocity.is_zero_approx()) {
        const Vector3 resolved_velocity = body->get_velocity();
        knockback_velocity = Vector3(resolved_velocity.x, 0, resolved_velocity.z)
                .move_toward(Vector3(), 18.0f * delta);
    }
    if (!animate) {
        return;
    }

    if (!facing.is_zero_approx()) {
        Vector3 rotation = rig->get_rotation();
        rotation.y = Math::lerp_angle(rotation.y, facing_yaw(Vector3(facing.x, 0, facing.y)), weight);
        rig->set_rotation(rotation);
    }
    update_backpedal_correction(delta);
    update_aim();
    recoil_left = MAX(recoil_left - delta, 0.0f);
    update_animation(locked);
}

void HumanoidController3D::set_aim_error(float degrees) {
    aim_error_degrees = MAX(0.0f, degrees);
}

bool HumanoidController3D::react_to_hit(const Vector3 &direction, float knockback_speed, float recovery_seconds) {
    if (hit_recovery_left > 0.0f || !rig || !rig->get_torso_player()) {
        return false;
    }
    apply_knockback(direction, knockback_speed);
    cancel_action();
    const bool playing = play_action(Action::REACTION, "damage0", 0.08, HIT_REACTION_SPEED);
    const float duration = playing ? rig->get_torso_player()->get_current_animation_length() / HIT_REACTION_SPEED : 0.0f;
    hit_recovery_left = duration + MAX(0.0f, recovery_seconds);
    return true;
}

void HumanoidController3D::apply_knockback(const Vector3 &direction, float speed) {
    const Vector3 horizontal(direction.x, 0, direction.z);
    if (speed > 0.0f && !horizontal.is_zero_approx()) {
        knockback_velocity = horizontal.normalized() * speed;
    }
}

bool HumanoidController3D::attack() {
    if (!rig || action != Action::LOCOMOTION) {
        return false;
    }
    const Vector3 direction = get_attack_direction();
    if (has_firearm()) {
        if (cooldown_left > 0.0f) {
            return false;
        }
        cooldown_left = cooldown;
        const float follow_up = recoil_left / RECOIL_DURATION;
        const float spread = spread_degrees + aim_error_degrees;
        const float error = UtilityFunctions::randf_range(-spread, spread);
        const float kick = recoil_yaw * follow_up;
        recoil_yaw = UtilityFunctions::randf_range(-recoil_strength, recoil_strength) * 0.12f;
        recoil_left = RECOIL_DURATION;
        emit_signal("fire", direction.rotated(Vector3(0, 1, 0), Math::deg_to_rad(error) + kick));
        return true;
    }

    const WeaponStyle attack_style = active_style();
    if (attack_style == BAT) {
        if (!play_action(Action::BAT_SWING, "swing_bat", 0.08)) {
            return false;
        }
        Vector3 rotation = rig->get_rotation();
        rotation.y = facing_yaw(direction);
        rig->set_rotation(rotation);
        rig->play_legs("swing_bat", 0.08, 1.0);
    } else {
        static const char *punches[] = {"punch0", "punch1", "punch2", "punch_right_jab"};
        static const char *knife_attacks[] = {"melee1", "melee2", "punch_right_jab"};
        const char *clip = attack_style == KNIFE ? knife_attacks[UtilityFunctions::randi_range(0, 2)] :
                punches[UtilityFunctions::randi_range(0, 3)];
        if (!play_action(Action::MELEE, clip, 0.08)) {
            return false;
        }
    }
    swing_direction = direction;
    impact_style = attack_style;
    impact_pending = true;
    return true;
}

bool HumanoidController3D::throw_item(Node3D *item) {
    if (!rig || action != Action::LOCOMOTION || !item ||
            (!rig->is_ancestor_of(item)) || !play_action(Action::THROW, "throw", 0.08, THROW_SPEED)) {
        return false;
    }
    thrown_item_id = ObjectID(item->get_instance_id());
    swing_direction = get_attack_direction();
    Vector3 rotation = rig->get_rotation();
    rotation.y = facing_yaw(swing_direction);
    rig->set_rotation(rotation);
    rig->clear_aim();
    if (item != get_weapon()) {
        rig->mount_weapon(item, true);
    }
    update_weapon_mount();
    rig->play_legs("throw", 0.08, THROW_SPEED);
    return true;
}

bool HumanoidController3D::interact(const StringName &clip, const Vector3 &direction) {
    if (!rig || action != Action::LOCOMOTION) {
        return false;
    }
    float event_time = 0.0f;
    double speed = 1.5;
    if (clip == StringName("pickup0")) {
        event_time = 1.35f;
        speed = 2.2;
    } else if (clip == StringName("button")) {
        event_time = 0.65f;
    } else if (clip == StringName("door")) {
        event_time = 1.05f;
        speed = 2.0;
    } else if (clip == StringName("push")) {
        event_time = 0.45f;
    } else if (clip != StringName("talk0") && clip != StringName("talk1") &&
            clip != StringName("talk2") && clip != StringName("talk3")) {
        return false;
    }
    if (!play_action(Action::INTERACT, clip, 0.1, speed)) {
        return false;
    }
    const Vector3 horizontal(direction.x, 0, direction.z);
    swing_direction = horizontal.is_zero_approx() ? get_attack_direction() : horizontal.normalized();
    rig->clear_aim();
    rig->play_legs(clip, 0.1, speed);
    interaction_time = 0.0f;
    interaction_event_time = event_time / speed;
    interaction_duration = rig->get_torso_player()->get_current_animation_length() / speed;
    impact_pending = event_time > 0.0f;
    if (!impact_pending) {
        emit_signal("interaction_contact", clip);
    }
    return true;
}

void HumanoidController3D::update_weapon_mount() {
    if (rig) {
        rig->set_weapon_aiming(aiming || action == Action::THROW || (style == UNARMED && get_weapon()));
    }
}

void HumanoidController3D::update_interaction(float delta) {
    if (action == Action::THROW && thrown_item_id.is_valid() &&
            rig->get_torso_player()->get_current_animation_position() >= THROW_RELEASE_TIME) {
        const ObjectID released = thrown_item_id;
        thrown_item_id = ObjectID();
        auto *item = Object::cast_to<Node3D>(ObjectDB::get_instance(released));
        if (released == weapon_id) {
            weapon_id = ObjectID();
            style = UNARMED;
            rig->configure_aim_weapon(nullptr, Transform3D(), Transform3D());
        }
        if (item) {
            emit_signal("item_released", item, swing_direction);
        }
    } else if (action == Action::INTERACT) {
        interaction_time += delta;
        if (impact_pending && interaction_time >= interaction_event_time) {
            impact_pending = false;
            emit_signal("interaction_contact", action_clip);
        }
        if (action == Action::INTERACT && interaction_time >= interaction_duration) {
            cancel_action();
        }
    }
}

void HumanoidController3D::reload() {
    if (rig && has_firearm() && action == Action::LOCOMOTION) {
        recoil_left = 0.0f;
        play_action(Action::RELOAD, "reload", 0.12);
    }
}

void HumanoidController3D::jump() {
    if (body && body->is_on_floor()) {
        Vector3 velocity = body->get_velocity();
        velocity.y = 4.245f;
        body->set_velocity(velocity);
    }
}

bool HumanoidController3D::play_reaction(const StringName &clip) {
    if (!rig) {
        return false;
    }
    cancel_action();
    return play_action(Action::REACTION, clip, 0.08);
}

double HumanoidController3D::play_defeat() {
    if (!rig || !rig->get_torso_player() || !rig->get_legs_player()) {
        return 0.0;
    }
    static constexpr const char *CLIPS[] = {"death0", "death1", "death2", "death3", "death4", "death5"};
    AnimationPlayer *torso = rig->get_torso_player();
    AnimationPlayer *legs = rig->get_legs_player();
    StringName available[6];
    int count = 0;
    for (const char *name : CLIPS) {
        const StringName clip(name);
        const StringName path = qualified(clip);
        if (torso->has_animation(path) && legs->has_animation(path)) {
            available[count++] = clip;
        }
    }
    if (count == 0) {
        return 0.0;
    }
    const StringName clip = available[UtilityFunctions::randi_range(0, count - 1)];
    set_input(Vector2(), Vector2(), false);
    rig->clear_aim();
    if (!play_reaction(clip)) {
        return 0.0;
    }
    rig->play_legs(clip, 0.08, 1.0);
    return torso->get_current_animation_length();
}

Vector3 HumanoidController3D::get_attack_direction() const {
    if (aiming) {
        return Vector3(aim.x, 0, aim.y).normalized();
    }
    if (rig) {
        const Vector3 forward = -rig->get_global_basis().get_column(2);
        return Vector3(forward.x, 0, forward.z).normalized();
    }
    return Vector3(0, 0, -1);
}

void HumanoidController3D::update_aim() {
    const AimProfile &profile = AIM_PROFILES[active_style()];
    if (action == Action::THROW || action == Action::INTERACT || (!aiming && recoil_left <= 0.0f) || !profile.use_chest ||
            (action == Action::MELEE && action_clip == StringName("punch_right_jab"))) {
        rig->clear_aim();
        return;
    }
    const Vector3 direction = get_attack_direction();
    const float yaw = reversed ? profile.reversed_yaw : profile.normal_yaw;
    const float weight = recoil_left / RECOIL_DURATION;
    const Vector2 adjusted = Vector2(direction.x, direction.z).rotated(
            Math::deg_to_rad(yaw) + aim_correction.x - recoil_yaw * weight * weight);
    const float height = (reversed ? profile.reversed_height : profile.normal_height) +
            aim_correction.y + recoil_strength * weight * weight;
    rig->set_aim_target(body->get_global_position() + Vector3(adjusted.x * AIM_DISTANCE, height, adjusted.y * AIM_DISTANCE));
}

void HumanoidController3D::update_backpedal_correction(float delta) {
    if (!has_firearm() || !body->is_on_floor() || move.dot(aim) > -0.701f) {
        aim_correction = Vector2();
        return;
    }
    Node3D *weapon = get_weapon();
    if (!weapon || action != Action::LOCOMOTION || recoil_left > 0.0f ||
            rig->get_torso_player()->get_current_animation() != qualified(StringName(String("aim_") + AIM_PROFILES[style].stance))) {
        return;
    }
    const Vector3 barrel = -weapon->get_global_basis().get_column(2).normalized();
    const Vector2 horizontal(barrel.x, barrel.z);
    if (horizontal.normalized().dot(aim) < 0.95f) {
        return;
    }
    const float weight = 1.0f - Math::exp(-24.0f * delta);
    aim_correction.x = Math::clamp(aim_correction.x + horizontal.angle_to(aim) * weight, -0.2f, 0.2f);
    aim_correction.y = Math::clamp(aim_correction.y - barrel.y * AIM_DISTANCE * weight, -0.55f, 0.55f);
}

void HumanoidController3D::update_animation(bool bat_swing) {
    if (bat_swing) {
        return;
    }
    const AimProfile &profile = AIM_PROFILES[active_style()];
    StringName torso_clip = move.is_zero_approx() ? "idle_unarmed" : "fwd_unarmed";
    if (!body->is_on_floor()) {
        torso_clip = "fall";
        play_loop(rig->get_legs_player(), torso_clip);
    } else if (aiming) {
        const Vector2 forward = aim.normalized();
        const Vector2 right(-forward.y, forward.x);
        const StringName legs_stance = active_style() == BAT && !move.is_zero_approx() ? "twohand" : profile.stance;
        rig->play_locomotion(Vector2(move.dot(right), move.dot(forward)), legs_stance, 0.15);
    } else {
        play_loop(rig->get_legs_player(), torso_clip);
    }
    if (action == Action::LOCOMOTION) {
        if (aiming || recoil_left > 0.0f) {
            torso_clip = String("aim_") + profile.stance;
        }
        if (style == UNARMED && get_weapon()) {
            torso_clip = "carry";
        }
        play_loop(rig->get_torso_player(), torso_clip);
    }
}

void HumanoidController3D::update_impact(AnimationPlayer *torso) {
    if (!impact_pending || (action != Action::MELEE && action != Action::BAT_SWING)) {
        return;
    }
    const float impact_time = action == Action::BAT_SWING ? BAT_IMPACT_TIME : 0.18f;
    if (torso->get_current_animation_position() >= impact_time) {
        impact_pending = false;
        emit_signal("melee_attack", swing_direction, impact_style);
    }
}

bool HumanoidController3D::play_action(Action next, const StringName &clip, double blend, double speed) {
    if (!rig->play_torso(clip, blend, speed)) {
        return false;
    }
    action = next;
    action_clip = clip;
    return true;
}

void HumanoidController3D::cancel_action() {
    if (rig && action != Action::LOCOMOTION) {
        if (rig->get_torso_player()) {
            rig->get_torso_player()->stop();
        }
        if ((action == Action::BAT_SWING || action == Action::THROW || action == Action::INTERACT) && rig->get_legs_player()) {
            rig->get_legs_player()->stop();
        }
    }
    action = Action::LOCOMOTION;
    impact_pending = false;
    thrown_item_id = ObjectID();
    update_weapon_mount();
}

void HumanoidController3D::on_torso_finished(const StringName &clip) {
    if (action == Action::LOCOMOTION || clip != qualified(action_clip)) {
        return;
    }
    if (action == Action::THROW || action == Action::INTERACT) {
        cancel_action();
    } else {
        action = Action::LOCOMOTION;
        impact_pending = false;
    }
}

bool HumanoidController3D::has_firearm() const {
    return aiming && (style == PISTOL || style == LONG_GUN);
}

HumanoidController3D::WeaponStyle HumanoidController3D::active_style() const {
    return aiming ? style : UNARMED;
}

Node3D *HumanoidController3D::get_weapon() const {
    return weapon_id.is_valid() ? Object::cast_to<Node3D>(ObjectDB::get_instance(weapon_id)) : nullptr;
}

void HumanoidController3D::set_move_speed(float speed) {
    move_speed = MAX(speed, 0.0f);
}

void HumanoidController3D::_bind_methods() {
    ClassDB::bind_method(D_METHOD("configure", "body", "rig"), &HumanoidController3D::configure);
    ClassDB::bind_method(D_METHOD("configure_weapon", "weapon", "style", "cooldown", "automatic", "recoil", "reversed", "spread_degrees"),
            &HumanoidController3D::configure_weapon, DEFVAL(0.0f));
    ClassDB::bind_method(D_METHOD("set_input", "move", "aim", "trigger_held"), &HumanoidController3D::set_input, DEFVAL(false));
    ClassDB::bind_method(D_METHOD("step", "delta", "animate"), &HumanoidController3D::step, DEFVAL(true));
    ClassDB::bind_method(D_METHOD("set_aim_error", "degrees"), &HumanoidController3D::set_aim_error);
    ClassDB::bind_method(D_METHOD("react_to_hit", "direction", "knockback_speed", "recovery_seconds"),
            &HumanoidController3D::react_to_hit);
    ClassDB::bind_method(D_METHOD("apply_knockback", "direction", "speed"), &HumanoidController3D::apply_knockback);
    ClassDB::bind_method(D_METHOD("throw_item", "item"), &HumanoidController3D::throw_item);
    ClassDB::bind_method(D_METHOD("is_throwing"), &HumanoidController3D::is_throwing);
    ClassDB::bind_method(D_METHOD("interact", "clip", "direction"), &HumanoidController3D::interact);
    ClassDB::bind_method(D_METHOD("attack"), &HumanoidController3D::attack);
    ClassDB::bind_method(D_METHOD("reload"), &HumanoidController3D::reload);
    ClassDB::bind_method(D_METHOD("jump"), &HumanoidController3D::jump);
    ClassDB::bind_method(D_METHOD("play_reaction", "clip"), &HumanoidController3D::play_reaction);
    ClassDB::bind_method(D_METHOD("play_defeat"), &HumanoidController3D::play_defeat);
    ClassDB::bind_method(D_METHOD("get_attack_direction"), &HumanoidController3D::get_attack_direction);
    ClassDB::bind_method(D_METHOD("is_aiming"), &HumanoidController3D::is_aiming);
    ClassDB::bind_method(D_METHOD("set_move_speed", "speed"), &HumanoidController3D::set_move_speed);
    ClassDB::bind_method(D_METHOD("get_move_speed"), &HumanoidController3D::get_move_speed);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "move_speed"), "set_move_speed", "get_move_speed");
    ADD_SIGNAL(MethodInfo("aiming_changed", PropertyInfo(Variant::BOOL, "aiming")));
    ADD_SIGNAL(MethodInfo("item_released", PropertyInfo(Variant::OBJECT, "item", PROPERTY_HINT_NODE_TYPE, "Node3D"),
            PropertyInfo(Variant::VECTOR3, "direction")));
    ADD_SIGNAL(MethodInfo("interaction_contact", PropertyInfo(Variant::STRING_NAME, "clip")));
    ADD_SIGNAL(MethodInfo("fire", PropertyInfo(Variant::VECTOR3, "direction")));
    ADD_SIGNAL(MethodInfo("melee_attack", PropertyInfo(Variant::VECTOR3, "direction"), PropertyInfo(Variant::INT, "style")));
    BIND_ENUM_CONSTANT(UNARMED);
    BIND_ENUM_CONSTANT(KNIFE);
    BIND_ENUM_CONSTANT(PISTOL);
    BIND_ENUM_CONSTANT(LONG_GUN);
    BIND_ENUM_CONSTANT(BAT);
}

} // namespace godot
