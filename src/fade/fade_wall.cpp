#include "fade/fade_wall.hpp"
#include "fade/geometry.hpp"

#include "godot_cpp/classes/engine.hpp"
// #include "godot_cpp/classes/resource_loader.hpp"
#include "godot_cpp/classes/camera3d.hpp"
#include "godot_cpp/classes/mesh.hpp"
#include "godot_cpp/classes/mesh_instance3d.hpp"
#include "godot_cpp/classes/object.hpp"
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
using namespace fadegeo;

void FadeWall3D::check_fade(Node3D * target, Camera3D * camera, float max_dist, float fade_speed, float min_alpha)
{
    // // TODO: this is fade floor logic. can prob skip by just adding to fade_obj group
    // if(target->get_global_position().y + TOP_MARGIN < get_global_position().y) {
    //     _fade_target = min_alpha;
    //     set_visible(false);
    //     return;
    // }
    // set_visible(true);
    // // TODO: unset fade if doing this

    Vector3 tp = target->get_global_position();
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
    if(p == Camera3D::PROJECTION_PERSPECTIVE) {
        Vector2 cam2 = Vector2(ct.origin.x, ct.origin.z);
        if(projected_outside(cam2, ply2, _center2d) ||
            !segment_hits_aabb(cam2, ply2, _aabb_min, _aabb_max)) {
            _fade_target = 1.0f;
        } else {
            _fade_target = min_alpha;
        }
    } else if(p == Camera3D::PROJECTION_ORTHOGONAL) {
        Vector3 fwd = -ct.basis.get_column(2);
        fwd.y = 0.0f;
        float yaw = Math::atan2(fwd.x, fwd.z);

        // pillar? skip back/front test
        if(!_wall_normal.is_zero_approx()) {
            Vector2 rel_norm = _wall_normal.rotated(-yaw);
            if((ply2 - _center2d).dot(rel_norm.rotated(camera->get_rotation().y)) <= 0.0f) {
                _fade_target = 1.0f;
                return;
            }
        }

        Vector2 proj_dir = Vector2(fwd.x, fwd.z).normalized();
        Vector2 a = ply2 - proj_dir * 9999.0f;
        Vector2 b = ply2 + proj_dir * 9999.0f;
        
        if(segment_hits_aabb(a, b, _aabb_min, _aabb_max)) {
            _fade_target = min_alpha;
        } else {
            _fade_target = 1.0f;
        }
    } else {
        UtilityFunctions::push_error("[FadeWall] ", get_name(), " check_fade : unsupported camera type");
    }
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
        UtilityFunctions::push_error("[FadeWall] ", get_name(), " _on_ready : MeshInstance3D not found. abort");
        return;
    }

    _fade_mat.instantiate();
    _fade_shader.instantiate();
    _fade_shader->set_code(R"(
shader_type spatial;
// render_mode blend_mix, depth_draw_alpha_prepass;

uniform float opacity : hint_range(0.0, 1.0) = 1.0;

uniform bool use_texture = false;
uniform sampler2D albedo_tex : source_color;
uniform vec4 albedo_color : source_color = vec4(1.0);

void fragment() {
    vec4 base = use_texture
        ? texture(albedo_tex, UV)
        : albedo_color;

    ALBEDO = base.rgb;
    ALPHA  = opacity;
}
    )");
    _fade_mat->set_shader(_fade_shader);
    if(!_fade_mat.is_valid()) {
        UtilityFunctions::push_error("[FadeWall] ", get_name(), "_on_ready : _fade_mat invalid. abort");
        return;
    }

    // TODO set render priority

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
}
