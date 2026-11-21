#include "humanoid/humanoid_rig_3d.hpp"

#include "godot_cpp/classes/animation_player.hpp"
#include "godot_cpp/classes/bone_attachment3d.hpp"
#include "godot_cpp/classes/look_at_modifier3d.hpp"
#include "godot_cpp/classes/skeleton3d.hpp"
#include "godot_cpp/core/class_db.hpp"
#include "godot_cpp/core/math.hpp"
#include "godot_cpp/variant/utility_functions.hpp"

namespace godot {
namespace {

bool play_clip(AnimationPlayer *player, const StringName &clip, double blend, double speed) {
    const StringName qualified = String("main/") + String(clip);
    if (!player || !player->has_animation(qualified)) {
        return false;
    }
    player->play(qualified, blend, speed);
    return true;
}

AnimationPlayer *add_layer(Node3D *model, const StringName &name, const Ref<AnimationLibrary> &library) {
    auto *player = memnew(AnimationPlayer);
    player->set_name(name);
    player->set_root_node(NodePath(".."));
    player->add_animation_library("main", library);
    model->add_child(player);
    return player;
}

BoneAttachment3D *add_socket(Skeleton3D *skeleton, const StringName &name, const StringName &bone) {
    auto *socket = memnew(BoneAttachment3D);
    socket->set_name(name);
    socket->set_bone_name(bone);
    skeleton->add_child(socket);
    return socket;
}

} // namespace

bool HumanoidRig3D::configure(const Ref<PackedScene> &model_scene, const Ref<AnimationLibrary> &torso_library,
        const Ref<AnimationLibrary> &legs_library, bool faces_positive_z,
        const StringName &chest_bone, const StringName &hand_bone, const StringName &stow_bone) {
    if (!is_inside_tree() || model_scene.is_null() || torso_library.is_null() || legs_library.is_null()) {
        UtilityFunctions::push_warning("HumanoidRig3D needs to be in the scene tree with a model and both animation libraries.");
        return false;
    }

    Node *instance = model_scene->instantiate();
    if (!instance) {
        return false;
    }
    Node3D *new_model = Object::cast_to<Node3D>(instance);
    const TypedArray<Node> skeletons = instance->find_children("*", "Skeleton3D", true, false);
    Skeleton3D *new_skeleton = skeletons.size() == 1 ? Object::cast_to<Skeleton3D>(skeletons[0]) : nullptr;
    if (!new_model || !new_skeleton || new_skeleton->find_bone(chest_bone) < 0 ||
            new_skeleton->find_bone(hand_bone) < 0 || new_skeleton->find_bone(stow_bone) < 0) {
        UtilityFunctions::push_warning("HumanoidRig3D needs a Node3D model with one skeleton and valid chest, hand, and stow bones.");
        memdelete(instance);
        return false;
    }

    for (const Variant &node : instance->find_children("*", "AnimationPlayer", true, false)) {
        auto *imported = Object::cast_to<AnimationPlayer>(node);
        imported->set_autoplay("");
        imported->set_active(false);
    }

    auto *new_torso = add_layer(new_model, "TorsoPlayer", torso_library);
    auto *new_legs = add_layer(new_model, "LegsPlayer", legs_library);
    auto *new_hand = add_socket(new_skeleton, "HandSocket", hand_bone);
    auto *new_stow = add_socket(new_skeleton, "StowSocket", stow_bone);

    auto *new_target = memnew(Node3D);
    new_target->set_name("AimTarget");
    new_model->add_child(new_target);

    auto *new_aim = memnew(LookAtModifier3D);
    new_aim->set_name("ChestAim");
    new_aim->set_active(false);
    new_aim->set_bone_name(chest_bone);
    new_aim->set_forward_axis(faces_positive_z ? SkeletonModifier3D::BONE_AXIS_PLUS_Z : SkeletonModifier3D::BONE_AXIS_MINUS_Z);
    new_skeleton->add_child(new_aim);

    if (faces_positive_z) {
        new_model->rotate_y(Math::deg_to_rad(180.0f));
    }
    add_child(new_model);
    new_aim->set_target_node(new_aim->get_path_to(new_target));
    if (model) {
        // Keep gear attached across a model swap without recreating the gear nodes.
        while (hand->get_child_count() > 0) {
            hand->get_child(0)->reparent(new_hand, false);
        }
        while (stow->get_child_count() > 0) {
            stow->get_child(0)->reparent(new_stow, false);
        }
        remove_child(model);
        model->queue_free();
    }

    model = new_model;
    skeleton = new_skeleton;
    torso = new_torso;
    legs = new_legs;
    hand = new_hand;
    stow = new_stow;
    aim = new_aim;
    aim_target = new_target;
    torso->connect("animation_finished", callable_mp(this, &HumanoidRig3D::on_torso_finished));
    return true;
}

bool HumanoidRig3D::play_torso(const StringName &clip, double blend, double speed) {
    return play_clip(torso, clip, blend, speed);
}

bool HumanoidRig3D::play_legs(const StringName &clip, double blend, double speed) {
    return play_clip(legs, clip, blend, speed);
}

bool HumanoidRig3D::play_locomotion(const Vector2 &local_motion, const StringName &stance, double blend) {
    String prefix = "aim_";
    if (Math::abs(local_motion.x) > Math::abs(local_motion.y)) {
        prefix = local_motion.x > 0.0f ? "right_" : "left_";
    } else if (!Math::is_zero_approx(local_motion.y)) {
        prefix = local_motion.y > 0.0f ? "fwd_" : "back_";
    }
    const String clip = prefix + String(stance);
    if (legs && legs->is_playing() && legs->get_current_animation() == StringName(String("main/") + clip)) {
        return true;
    }
    return play_legs(StringName(clip), blend, 1.0);
}

bool HumanoidRig3D::mount_weapon(Node3D *weapon, bool equipped) {
    BoneAttachment3D *socket = equipped ? hand : stow;
    if (!weapon || !socket || weapon == socket || weapon->is_ancestor_of(socket)) {
        return false;
    }
    if (weapon->get_parent() == socket) {
        return true;
    }
    if (weapon->get_parent()) {
        weapon->reparent(socket, false);
    } else {
        socket->add_child(weapon);
    }
    return weapon->get_parent() == socket;
}

void HumanoidRig3D::set_aim_target(const Vector3 &world_position) {
    if (aim) {
        aim_target->set_global_position(world_position);
        aim->set_active(true);
    }
}

void HumanoidRig3D::clear_aim() {
    if (aim) {
        aim->set_active(false);
    }
}

Skeleton3D *HumanoidRig3D::get_skeleton() const { return skeleton; }
AnimationPlayer *HumanoidRig3D::get_torso_player() const { return torso; }
AnimationPlayer *HumanoidRig3D::get_legs_player() const { return legs; }
BoneAttachment3D *HumanoidRig3D::get_hand_socket() const { return hand; }
BoneAttachment3D *HumanoidRig3D::get_stow_socket() const { return stow; }

void HumanoidRig3D::on_torso_finished(const StringName &clip) {
    emit_signal("torso_animation_finished", clip);
}

void HumanoidRig3D::_bind_methods() {
    ClassDB::bind_method(D_METHOD("configure", "model_scene", "torso_library", "legs_library", "faces_positive_z",
                                 "chest_bone", "hand_bone", "stow_bone"),
            &HumanoidRig3D::configure, DEFVAL(false), DEFVAL(StringName("Chest")), DEFVAL(StringName("RightHand")), DEFVAL(StringName("Spine")));
    ClassDB::bind_method(D_METHOD("play_torso", "clip", "blend", "speed"), &HumanoidRig3D::play_torso, DEFVAL(-1.0), DEFVAL(1.0));
    ClassDB::bind_method(D_METHOD("play_legs", "clip", "blend", "speed"), &HumanoidRig3D::play_legs, DEFVAL(-1.0), DEFVAL(1.0));
    ClassDB::bind_method(D_METHOD("play_locomotion", "local_motion", "stance", "blend"), &HumanoidRig3D::play_locomotion, DEFVAL(0.15));
    ClassDB::bind_method(D_METHOD("mount_weapon", "weapon", "equipped"), &HumanoidRig3D::mount_weapon);
    ClassDB::bind_method(D_METHOD("set_aim_target", "world_position"), &HumanoidRig3D::set_aim_target);
    ClassDB::bind_method(D_METHOD("clear_aim"), &HumanoidRig3D::clear_aim);
    ClassDB::bind_method(D_METHOD("get_skeleton"), &HumanoidRig3D::get_skeleton);
    ClassDB::bind_method(D_METHOD("get_torso_player"), &HumanoidRig3D::get_torso_player);
    ClassDB::bind_method(D_METHOD("get_legs_player"), &HumanoidRig3D::get_legs_player);
    ClassDB::bind_method(D_METHOD("get_hand_socket"), &HumanoidRig3D::get_hand_socket);
    ClassDB::bind_method(D_METHOD("get_stow_socket"), &HumanoidRig3D::get_stow_socket);
    ADD_SIGNAL(MethodInfo("torso_animation_finished", PropertyInfo(Variant::STRING_NAME, "clip")));
}

} // namespace godot
