#pragma once

#include "godot_cpp/classes/scene_tree.hpp"
#include "godot_cpp/classes/static_body3d.hpp"
#include <godot_cpp/core/class_db.hpp>

namespace godot {

class FadeFloor3D : public StaticBody3D {
    GDCLASS(FadeFloor3D, StaticBody3D);

public:
    FadeFloor3D() = default;
    ~FadeFloor3D() override = default;
    void _notification(int p_what);

    void check_fade(Node3D * target, float max_dist=3.0f);
    static void check_fade_floors(SceneTree * tree, Node3D * target, float max_dist=3.0f);
    static void check_fade_objects(SceneTree * tree, Node3D * target, float max_dist=3.0f);

    void set_global_y_margin(float m);
    float get_global_y_margin() const;
    void set_y_margin_override(float m);
    float get_y_margin_override() const;
    void set_margin_override_enable(bool m);
    bool get_margin_override_enable() const;

protected:
    static void _bind_methods();

private:

    static float s_global_y_margin;

    // exports
    float y_margin_override = 0.0f;
    bool y_margin_override_enable = false;

    // internals
    Vector2 _aabb_min;
    Vector2 _aabb_max;

    float distance_to_footprint(const Vector3 &position) const;
    bool should_hide(const Vector3 &target, float max_dist) const;
    void _on_ready();

};

}
