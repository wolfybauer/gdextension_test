#pragma once

#include "godot_cpp/classes/shader_material.hpp"
#include "godot_cpp/classes/scene_tree.hpp"
#include "godot_cpp/classes/node3d.hpp"
#include "godot_cpp/classes/camera3d.hpp"
#include "godot_cpp/classes/static_body3d.hpp"
#include "godot_cpp/variant/vector2.hpp"
#include <godot_cpp/classes/ref.hpp>
#include <godot_cpp/classes/material.hpp>
#include <godot_cpp/core/class_db.hpp>

#define FADE_SHADER_PATH "res://gde_test/assets/fade.gdshader"

namespace godot {

class FadeWall3D : public StaticBody3D {
    GDCLASS(FadeWall3D, StaticBody3D);

public:
    FadeWall3D() = default;
    ~FadeWall3D() override = default;
    void _notification(int p_what);

    void check_fade(Node3D * target, Camera3D * camera, float max_dist=3.0f, float fade_speed=5.0f, float min_alpha=0.25f);
    static void check_fade_walls(SceneTree * tree, Node3D * target, Camera3D * camera, float max_dist=3.0f, float fade_speed=5.0f, float min_alpha=0.25f);

    
    void set_global_y_margin(float m);
    float get_global_y_margin() const;
    void set_y_margin_override(float m);
    float get_y_margin_override() const;
    void set_margin_override_enable(bool m);
    bool get_margin_override_enable() const;

    void set_global_render_priority(int lvl);
    int get_global_render_priority() const;
    void set_render_priority_override(int lvl);
    int get_render_priority_override() const;
    void set_priority_override_enable(bool m);
    bool get_priority_override_enable() const;

protected:
    static void _bind_methods();

    
private:

    static int s_global_render_priority;
    static float s_global_y_margin;

    // exports
    Ref<ShaderMaterial> _fade_mat;
    Ref<Shader> _fade_shader;
    int render_prio_override = 0;
    bool render_prio_override_enable = false;
    float y_margin_override = 0.0f;
    bool y_margin_override_enable = false;

    // internals
    Vector2 _aabb_min;
    Vector2 _aabb_max;
    Vector2 _center2d;
    Vector2 _wall_normal;

    float _fade_amount = 1.0f;
    float _fade_target = 1.0f;
    float _fade_speed = 5.0f;
    float _target_dist = 0.0f;

    int * _render_prio = nullptr;
    float  * _y_margin = nullptr;

    // funcs
    void _on_ready();
    void _on_physics_process(float delta);
    void _apply_render_priority();
};

}
