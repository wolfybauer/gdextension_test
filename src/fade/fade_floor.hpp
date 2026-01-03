#pragma once

// #include "godot_cpp/classes/shader_material.hpp"
#include "godot_cpp/classes/scene_tree.hpp"
#include "godot_cpp/classes/static_body3d.hpp"
// #include "godot_cpp/variant/vector2.hpp"
#include <godot_cpp/classes/ref.hpp>
#include <godot_cpp/classes/material.hpp>
#include <godot_cpp/core/class_db.hpp>

class Node3D;
class Camer3D;

namespace godot {

class FadeFloor3D : public StaticBody3D {
    GDCLASS(FadeFloor3D, StaticBody3D);

public:
    FadeFloor3D() = default;
    ~FadeFloor3D() override = default;
    void _notification(int p_what);

    void check_fade(Node3D * target, float max_dist=3.0f);
    static void check_fade_objects(SceneTree * tree, Node3D * target, float max_dist=3.0f);

    void set_global_y_margin(float m);
    float get_global_y_margin() const;
    void set_y_margin_override(float m);
    float get_y_margin_override() const;
    void set_margin_override_enable(bool m);
    bool get_margin_override_enable() const;

protected:
    static void _bind_methods();

    // static int s_global_render_priority;

private:

    static float s_global_y_margin;
    static float s_lowest_floor_height;
    static float s_last_lowest_floor_height;

    // exports
    float y_margin_override = 0.0f;
    bool y_margin_override_enable = false;

    // internals
    Vector2 _aabb_min;
    Vector2 _aabb_max;
    Vector2 _center2d;
    Vector2 _wall_normal;

    float  * _y_margin = nullptr;

    // funcs
    void _on_ready();
    void _apply_render_priority();

};

}
