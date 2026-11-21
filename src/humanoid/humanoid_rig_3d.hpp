#pragma once

#include "godot_cpp/classes/animation_library.hpp"
#include "godot_cpp/classes/node3d.hpp"
#include "godot_cpp/classes/packed_scene.hpp"
#include "godot_cpp/core/object_id.hpp"

namespace godot {

class AnimationPlayer;
class BoneAttachment3D;
class LookAtModifier3D;
class Skeleton3D;

class HumanoidRig3D : public Node3D {
    GDCLASS(HumanoidRig3D, Node3D)

public:
    bool configure(const Ref<PackedScene> &model_scene, const Ref<AnimationLibrary> &torso_library,
            const Ref<AnimationLibrary> &legs_library, bool faces_positive_z,
            const StringName &chest_bone, const StringName &hand_bone, const StringName &stow_bone);
    bool play_torso(const StringName &clip, double blend, double speed);
    bool play_legs(const StringName &clip, double blend, double speed);
    bool play_locomotion(const Vector2 &local_motion, const StringName &stance, double blend);
    bool mount_weapon(Node3D *weapon, bool equipped);
    void configure_aim_weapon(Node3D *weapon, const Transform3D &grip, const Transform3D &stow_mount);
    void set_weapon_aiming(bool aiming);
    void set_aim_target(const Vector3 &world_position);
    void clear_aim();

    Skeleton3D *get_skeleton() const;
    AnimationPlayer *get_torso_player() const;
    AnimationPlayer *get_legs_player() const;
    BoneAttachment3D *get_hand_socket() const;
    BoneAttachment3D *get_stow_socket() const;

protected:
    static void _bind_methods();

private:
    Node3D *model = nullptr;
    Skeleton3D *skeleton = nullptr;
    AnimationPlayer *torso = nullptr;
    AnimationPlayer *legs = nullptr;
    BoneAttachment3D *hand = nullptr;
    BoneAttachment3D *stow = nullptr;
    LookAtModifier3D *aim = nullptr;
    Node3D *aim_target = nullptr;
    ObjectID aim_weapon_id;
    Transform3D weapon_grip;
    Transform3D weapon_stow;
    bool weapon_aiming = false;

    void update_aim_weapon_mount();
    void on_torso_finished(const StringName &clip);
};

} // namespace godot
