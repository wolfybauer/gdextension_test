#pragma once

#include "godot_cpp/classes/shader_material.hpp"
#include "godot_cpp/classes/static_body3d.hpp"
#include "godot_cpp/variant/vector2.hpp"
#include <godot_cpp/classes/ref.hpp>
#include <godot_cpp/classes/material.hpp>
#include <godot_cpp/core/class_db.hpp>

#define TOP_MARGIN 2.0f

class Node3D;
class Camer3D;

namespace godot {

class FadeFloor3D : public StaticBody3D {
    GDCLASS(FadeFloor3D, StaticBody3D);

public:
    FadeFloor3D() = default;
    ~FadeFloor3D() override = default;
    void _notification(int p_what);

    void check_fade(Node3D * target, Camera3D * camera, float max_dist=3.0f, float fade_speed=5.0f, float min_alpha=0.25f);

protected:
    static void _bind_methods();

    // static int s_global_render_priority;

private:

    // exports
    Ref<ShaderMaterial> _fade_mat;
    Ref<Shader> _fade_shader;
    int render_prio_override = 0;
    bool render_prio_override_enable = false;

    // internals
    Vector2 _aabb_min;
    Vector2 _aabb_max;
    Vector2 _center2d;
    Vector2 _wall_normal;

    float _fade_amount = 1.0f;
    float _fade_target = 1.0f;
    float _fade_speed = 5.0f;

    float _target_dist = 0.0f;

    // funcs
    void _on_ready();
};

}
