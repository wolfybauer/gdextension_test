#include "fade/fade_wall.hpp"
#include "fade/geometry.hpp"

#include "godot_cpp/classes/engine.hpp"
// #include "godot_cpp/classes/scene_tree.hpp"
#include "godot_cpp/classes/resource_loader.hpp"
#include "godot_cpp/classes/camera3d.hpp"
#include "godot_cpp/classes/mesh.hpp"
#include "godot_cpp/classes/mesh_instance3d.hpp"
#include "godot_cpp/classes/object.hpp"
#include "godot_cpp/classes/resource_loader.hpp"
#include "godot_cpp/classes/standard_material3d.hpp"
// #include "godot_cpp/classes/texture2d_array.hpp"
#include "godot_cpp/core/math.hpp"
#include "godot_cpp/classes/texture.hpp"
#include "godot_cpp/variant/transform3d.hpp"
#include "godot_cpp/variant/utility_functions.hpp"
#include "godot_cpp/variant/vector2.hpp"
#include "godot_cpp/variant/vector3.hpp"
#include <cmath>

using namespace godot;
using namespace fade_geometry;

int FadeWall3D::s_global_render_priority = 0;
float FadeWall3D::s_global_y_margin = DEFAULT_FADE_Y_MARGIN;

void FadeWall3D::set_opacity(float op) {
    op = Math::clamp<float>(op, 0.0f,1.0f);
    _fade_target = op;
}


void FadeWall3D::check_fade(Node3D * target, Camera3D * camera, float max_dist, float fade_speed, float min_alpha)
{
    Vector3 tp = target->get_global_position();
    Vector3 mypos = get_global_position();
    if(tp.y + (*_y_margin) < mypos.y) {
        if(max_dist > 0.0f) {
            Vector2 t2(tp.x, tp.z);

            // clamp target to floor AABB in XZ
            Vector2 closest(
                Math::clamp(t2.x, _aabb_min.x, _aabb_max.x),
                Math::clamp(t2.y, _aabb_min.y, _aabb_max.y)
            );

            float dist = t2.distance_to(closest);
            if (dist > max_dist) {
                _fade_target = 1.0f;
                // set_visible(true);
                return;
            }
        }
        _fade_target = 0.0f;
        // set_visible(false);
        return;
    }
    // set_visible(true);
    if((mypos.y + (*_y_margin)) < tp.y) {
        _fade_target = 1.0f;
        return;
    }

    Vector2 ply2 = Vector2(tp.x, tp.z);

    if(max_dist > 0.0f && !_wall_normal.is_zero_approx()) {
        _target_dist = (ply2 - _center2d).dot(_wall_normal);
        if(Math::abs<float>(_target_dist) > max_dist) {
            _fade_target = 1.0f;
            return;
        }
    }


    Transform3D ct = camera->get_global_transform();
    enum Camera3D::ProjectionType p = camera->get_projection();
    AABB wall_aabb = get_global_transform().xform(_local_aabb);

    if(p == Camera3D::PROJECTION_PERSPECTIVE) {
        // Vector2 cam2 = Vector2(ct.origin.x, ct.origin.z);
        // if(projected_outside(cam2, ply2, _center2d) ||
        //     !segment_hits_aabb(cam2, ply2, _aabb_min, _aabb_max)) {
        //     _fade_target = 1.0f;
        // } else {
        //     _fade_target = min_alpha;
        // }
        Vector3 ray_from = ct.origin;
        Vector3 ray_to   = tp;

        // does camera→player ray intersect the wall volume?
        if (wall_aabb.intersects_segment(ray_from, ray_to)) {
            _fade_target = min_alpha;   // occluding → fade
        } else {
            _fade_target = 1.0f;        // clear LOS → no fade
        }

    } else if(p == Camera3D::PROJECTION_ORTHOGONAL) {
        // Vector3 fwd = -ct.basis.get_column(2);
        // fwd.y = 0.0f;
        // float yaw = Math::atan2(fwd.x, fwd.z);

        // // pillar? skip back/front test
        // if(!_wall_normal.is_zero_approx()) {
        //     Vector2 rel_norm = _wall_normal.rotated(-yaw);
        //     if((ply2 - _center2d).dot(rel_norm.rotated(camera->get_global_rotation().y)) <= 0.0f) {
        //         _fade_target = 1.0f;
        //         return;
        //     }
        // }

        // Vector2 proj_dir = Vector2(fwd.x, fwd.z).normalized();
        // Vector2 a = ply2 - proj_dir * 9999.0f;
        // Vector2 b = ply2 + proj_dir * 9999.0f;
        
        // if(segment_hits_aabb(a, b, _aabb_min, _aabb_max)) {
        //     _fade_target = min_alpha;
        // } else {
        //     _fade_target = 1.0f;
        // }
        Vector3 dir = -ct.basis.get_column(2).normalized();
        Vector3 ray_from = tp - dir * 10000.0f;
        Vector3 ray_to   = tp + dir * 10000.0f;

        if (wall_aabb.intersects_segment(ray_from, ray_to)) {
            _fade_target = min_alpha;
        } else {
            _fade_target = 1.0f;
        }

    } else {
        UtilityFunctions::push_error("[FadeWall3D] ", get_name(), " check_fade : unsupported camera type");
    }
}

void FadeWall3D::check_fade_walls(SceneTree * tree, Node3D * target, Camera3D * camera, float max_dist, float fade_speed, float min_alpha) {
    if(!tree || !target || !camera) {
        UtilityFunctions::push_error("[FadeWall3D] check_fade_walls : tree or target or camera is null. abort");
        return;
    }
    tree->call_group("fade_wall", "check_fade", target, camera, max_dist, fade_speed, min_alpha);
}

void FadeWall3D::_on_ready() {
    if(Engine::get_singleton()->is_editor_hint()) {
		return;
	}

    MeshInstance3D * mesh_inst = nullptr;
    for(auto & v : get_children()) {
        mesh_inst = Object::cast_to<MeshInstance3D>(v);
        if(mesh_inst != nullptr) {
            break;
        }
    }
    if(mesh_inst == nullptr) {
        UtilityFunctions::push_error("[FadeWall3D] ", get_name(), " _on_ready : MeshInstance3D not found. abort");
        return;
    }

    _local_aabb = mesh_inst->get_aabb();

    _fade_mat.instantiate();
    _fade_shader = ResourceLoader::get_singleton()->load(FADE_SHADER_PATH);
    if(!_fade_shader.is_valid()) {
        UtilityFunctions::push_error("[FadeWall3D] ", get_name(), " _on_ready : error loading shader. abort");
        return;
    }
//     _fade_shader->set_code(R"(
// shader_type spatial;
// // render_mode blend_mix, depth_draw_alpha_prepass;

// uniform float opacity : hint_range(0.0, 1.0) = 1.0;

// uniform bool use_texture = false;
// uniform sampler2D albedo_tex : source_color;
// uniform vec4 albedo_color : source_color = vec4(1.0);

// void fragment() {
//     vec4 base = use_texture
//         ? texture(albedo_tex, UV)
//         : albedo_color;

//     ALBEDO = base.rgb;
//     ALPHA  = opacity;
// }
//     )");
    _fade_mat->set_shader(_fade_shader);
    if(!_fade_mat.is_valid()) {
        UtilityFunctions::push_error("[FadeWall3D] ", get_name(), "_on_ready : _fade_mat invalid. abort");
        return;
    }

    // assign render prio / y margin pointers
    _render_prio = render_prio_override_enable ? &render_prio_override : &s_global_render_priority;
    _y_margin = y_margin_override_enable ? &y_margin_override : &s_global_y_margin;
    _apply_render_priority();

    // get / set up the texture/color we'll be fading
    Ref<StandardMaterial3D> old = mesh_inst->get_mesh()->surface_get_material(0);
    if(old.is_valid()) {
        Ref<Texture> tex = old->get("albedo_texture");
        if(tex.is_valid()) {
            _fade_mat->set_shader_parameter("albedo_tex", tex);
            _fade_mat->set_shader_parameter("use_texture", true);
            _fade_mat->set_shader_parameter("albedo_color", Color(1,1,1,1));
        } else {
            _fade_mat->set_shader_parameter("albedo_color", old->get_albedo());
            _fade_mat->set_shader_parameter("use_texture", false);
        }
    }
    mesh_inst->set_surface_override_material(0, _fade_mat);

    precompute_bounds(mesh_inst, _aabb_min, _aabb_max, _center2d, _wall_normal); // TODO add error code

    add_to_group("fade_wall");
    // add_to_group("fade_obj");
    set_physics_process(true);
}

void FadeWall3D::_on_physics_process(float delta) {
    if(Engine::get_singleton()->is_editor_hint()) {
        return;
    }
    if(Math::is_equal_approx(_fade_amount, _fade_target)) {
        return;
    }
    if(!_fade_mat.is_valid()) {
        return;
    }
    _fade_amount = Math::lerp(_fade_amount, _fade_target, _fade_speed * delta);
    _fade_mat->set_shader_parameter("opacity", _fade_amount);
}

void FadeWall3D::_notification(int p_what) {
    switch (p_what) {
		case NOTIFICATION_READY: {
            _on_ready();
            break;
        }

        case NOTIFICATION_PHYSICS_PROCESS: {
            _on_physics_process((float)get_physics_process_delta_time());
            break;
        }
        default:
            break;
    }
}

void FadeWall3D::_bind_methods() {
    ClassDB::bind_method(D_METHOD("check_fade", "target", "camera", "max_dist", "fade_speed", "min_alpha"), &FadeWall3D::check_fade);
    ClassDB::bind_static_method("FadeWall3D", D_METHOD("check_fade_walls", "tree", "target", "camera", "max_dist", "fade_speed", "min_alpha"), &FadeWall3D::check_fade_walls);
    
    ClassDB::bind_method(D_METHOD("set_global_render_priority", "lvl"), &FadeWall3D::set_global_render_priority);
    ClassDB::bind_method(D_METHOD("get_global_render_priority"), &FadeWall3D::get_global_render_priority);

    ClassDB::bind_method(D_METHOD("set_render_priority_override", "lvl"), &FadeWall3D::set_render_priority_override);
    ClassDB::bind_method(D_METHOD("get_render_priority_override"), &FadeWall3D::get_render_priority_override);
    ClassDB::bind_method(D_METHOD("set_priority_override_enable", "en"), &FadeWall3D::set_priority_override_enable);
    ClassDB::bind_method(D_METHOD("get_priority_override_enable"), &FadeWall3D::get_priority_override_enable);

    ClassDB::bind_method(D_METHOD("set_global_y_margin", "height"), &FadeWall3D::set_global_y_margin);
    ClassDB::bind_method(D_METHOD("get_global_y_margin"), &FadeWall3D::get_global_y_margin);
    
    ClassDB::bind_method(D_METHOD("set_y_margin_override", "height"), &FadeWall3D::set_y_margin_override);
    ClassDB::bind_method(D_METHOD("get_y_margin_override"), &FadeWall3D::get_y_margin_override);
    ClassDB::bind_method(D_METHOD("set_margin_override_enable", "en"), &FadeWall3D::set_margin_override_enable);
    ClassDB::bind_method(D_METHOD("get_margin_override_enable"), &FadeWall3D::get_margin_override_enable);

    ADD_PROPERTY(
        PropertyInfo(
            Variant::INT,
            "rendering/global_render_priority"
        ),
        "set_global_render_priority",
        "get_global_render_priority"
    );
    
    ADD_PROPERTY(
        PropertyInfo(
            Variant::INT,
            "rendering/render_priority_override"
        ),
        "set_render_priority_override",
        "get_render_priority_override"
    );

    ADD_PROPERTY(
        PropertyInfo(
            Variant::BOOL,
            "rendering/priority_override_enable"
        ),
        "set_priority_override_enable",
        "get_priority_override_enable"
    );

    ADD_PROPERTY(
        PropertyInfo(
            Variant::FLOAT,
            "rendering/global_y_margin",
            PROPERTY_HINT_RANGE,
            "0.0,10.0,0.1"
        ),
        "set_global_y_margin",
        "get_global_y_margin"
    );

    ADD_PROPERTY(
        PropertyInfo(
            Variant::FLOAT,
            "rendering/y_margin_override",
            PROPERTY_HINT_RANGE,
            "0.0,10.0,0.1"
        ),
        "set_y_margin_override",
        "get_y_margin_override"
    );

    ADD_PROPERTY(
        PropertyInfo(
            Variant::BOOL,
            "rendering/margin_override_enable"
        ),
        "set_margin_override_enable",
        "get_margin_override_enable"
    );
}

void FadeWall3D::_apply_render_priority() {
    if (!_fade_mat.is_valid() || !_render_prio) {
        return;
    }
    _fade_mat->set_render_priority(*_render_prio);
}


void FadeWall3D::set_global_render_priority(int lvl) {
    s_global_render_priority = lvl;

    if(Engine::get_singleton()->is_editor_hint()) {
        return;
    }

    Array walls = get_tree()->get_nodes_in_group("fade_wall");
    for (int i = 0; i < walls.size(); i++) {
        FadeWall3D *fw = Object::cast_to<FadeWall3D>(walls[i]);
        if (!fw) {
            continue;
        }

        if (!fw->render_prio_override_enable) {
            fw->_render_prio = &s_global_render_priority;
            fw->_apply_render_priority();
        }
    }
}


int FadeWall3D::get_global_render_priority() const {
    return s_global_render_priority;
}

void FadeWall3D::set_render_priority_override(int lvl) {
    render_prio_override = lvl;
    if (render_prio_override_enable) {
        _apply_render_priority();
    }
}

int FadeWall3D::get_render_priority_override() const {
    return render_prio_override;
}

void FadeWall3D::set_priority_override_enable(bool e) {
    render_prio_override_enable = e;
    _render_prio = e ? &render_prio_override : &s_global_render_priority;
    _apply_render_priority();
}

bool FadeWall3D::get_priority_override_enable() const {
    return render_prio_override_enable;
}

void FadeWall3D::set_global_y_margin(float m) {
    s_global_y_margin = m;

    if(Engine::get_singleton()->is_editor_hint()) {
        return;
    }

    Array walls = get_tree()->get_nodes_in_group("fade_wall");
    for (int i = 0; i < walls.size(); i++) {
        FadeWall3D *fw = Object::cast_to<FadeWall3D>(walls[i]);
        if (!fw) {
            continue;
        }

        if (!fw->y_margin_override_enable) {
            fw->_y_margin = &s_global_y_margin;
        }
    }
}

float FadeWall3D::get_global_y_margin() const {
    return s_global_y_margin;
}

void FadeWall3D::set_y_margin_override(float m) {
    y_margin_override = m;
}

float FadeWall3D::get_y_margin_override() const {
    return y_margin_override;
}

void FadeWall3D::set_margin_override_enable(bool e) {
    y_margin_override_enable = e;
    _y_margin = e ? &y_margin_override : &s_global_y_margin;
}

bool FadeWall3D::get_margin_override_enable() const {
    return y_margin_override_enable;
}
